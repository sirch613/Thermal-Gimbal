import enum
import struct
import time
import logging
import usb.core
import usb.util

log = logging.getLogger(__name__)

# T2 Pro VID/PID (Cypress)
T2PRO_VID = 0x04b4
T2PRO_PID = 0x0100

class PseudoColorTypes(enum.IntEnum):
    PSEUDO_WHITE_HOT = 1
    PSEUDO_RESERVED = 2
    PSEUDO_IRON_RED = 3
    PSEUDO_RAINBOW_1 = 4
    PSEUDO_RAINBOW_2 = 5
    PSEUDO_RAINBOW_3 = 6
    PSEUDO_RED_HOT = 7
    PSEUDO_HOT_RED = 8
    PSEUDO_RAINBOW_4 = 9
    PSEUDO_RAINBOW_5 = 10
    PSEUDO_BLACK_HOT = 11

class PropTpdParams(enum.IntEnum):
    TPD_PROP_DISTANCE = 0   # 1/163.835 m, 0-32767, Distance
    TPD_PROP_TU = 1         # 1 K, 0-1024, Reflection temperature
    TPD_PROP_TA = 2         # 1 K, 0-1024, Atmospheric temperature
    TPD_PROP_EMS = 3        # 1/127, 0-127, Emissivity
    TPD_PROP_TAU = 4        # 1/127, 0-127, Atmospheric transmittance
    TPD_PROP_GAIN_SEL = 5   # binary, 0-1, Gain select (0=low, 1=high)

class DeviceInfoType(enum.IntEnum):
    DEV_INFO_CHIP_ID = 0
    DEV_INFO_FW_COMPILE_DATE = 1
    DEV_INFO_DEV_QUALIFICATION = 2
    DEV_INFO_IR_INFO = 3
    DEV_INFO_PROJECT_INFO = 4
    DEV_INFO_FW_BUILD_VERSION_INFO = 5
    DEV_INFO_GET_PN = 6
    DEV_INFO_GET_SN = 7
    DEV_INFO_GET_SENSOR_ID = 8

DeviceInfoType_len = [8, 8, 8, 26, 4, 50, 48, 16, 4]

class CmdDir(enum.IntFlag):
    GET = 0x0000
    SET = 0x4000

class CmdCode(enum.IntEnum):
    sys_reset_to_rom = 0x0805
    spi_transfer = 0x8201
    get_device_info = 0x8405
    pseudo_color = 0x8409
    shutter_vtemp = 0x840c
    prop_tpd_params = 0x8514
    cur_vtemp = 0x8b0d
    preview_start = 0xc10f
    preview_stop = 0x020f
    y16_preview_start = 0x010a
    y16_preview_stop = 0x020a

class T2ProCmd:
    _dev: usb.core.Device

    def __init__(self):
        # Retry logic for device finding
        for i in range(3):
            self._dev = usb.core.find(idVendor=T2PRO_VID, idProduct=T2PRO_PID)
            if self._dev: break
            time.sleep(0.1)

        if (self._dev is None):
            self._dev = usb.core.find(idVendor=0x0BDA, idProduct=0x5830)
            if self._dev:
                print("[INFO] Found P2Pro (VID 0x0BDA)")
            else:
                 # Don't crash immediately, let caller handle
                 raise FileNotFoundError(f"T2 Pro thermal module not found!")
        else:
            pass # Silent success
            # print(f"[INFO] Found T2 Pro (VID {T2PRO_VID:#06x})")

        # Detach kernel driver is risky if libuvc is using it.
        # We try to set configuration only if needed.
        try:
            if self._dev.is_kernel_driver_active(0):
                self._dev.detach_kernel_driver(0)
                print("[INFO] Detached kernel driver")
        except Exception:
             pass # Ignore errors, libuvc might own it
             
    def close(self):
        """Release usage of the device"""
        try:
            usb.util.dispose_resources(self._dev)
        except:
            pass
            
    def __enter__(self):
        return self
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.close()

    def _check_camera_ready(self) -> bool:
        """
        Checks if the camera is ready.
        """
        # Vendor request details from P2Pro-Viewer
        try:
            ret = self._dev.ctrl_transfer(0xC1, 0x44, 0x78, 0x200, 1, timeout=3000)
        except usb.core.USBError:
            return False
            
        if (ret[0] & 1 == 0 and ret[0] & 2 == 0):
            return True
        if (ret[0] & 0xFC != 0):
            # raise UserWarning(f"vdcmd status error {ret[0]:#X}")
            # Suppress warning to avoid spam, just return False
            return False
        return False

    def _block_until_camera_ready(self, timeout: int = 5) -> bool:
        start = time.time()
        while True:
            try:
                if (self._check_camera_ready()):
                    return True
            except Exception:
                pass 
            time.sleep(0.005)
            if (time.time() > start + timeout):
                return False

    def _long_cmd_write(self, cmd: int, p1: int, p2: int, p3: int = 0, p4: int = 0):
        data1 = struct.pack("<H", cmd)
        data1 += struct.pack(">HI", p1, p2)
        data2 = struct.pack(">II", p3, p4)
        
        self._dev.ctrl_transfer(0x41, 0x45, 0x78, 0x9d00, data1, timeout=3000)
        self._dev.ctrl_transfer(0x41, 0x45, 0x78, 0x1d08, data2, timeout=3000)
        self._block_until_camera_ready()

    def _long_cmd_read(self, cmd: int, p1: int, p2: int = 0, p3: int = 0, dataLen: int = 2):
        data1 = struct.pack("<H", cmd)
        data1 += struct.pack(">HI", p1, p2)
        data2 = struct.pack(">II", p3, dataLen)
        
        self._dev.ctrl_transfer(0x41, 0x45, 0x78, 0x9d00, data1, timeout=3000)
        self._dev.ctrl_transfer(0x41, 0x45, 0x78, 0x1d08, data2, timeout=3000)
        self._block_until_camera_ready()
        
        res = self._dev.ctrl_transfer(0xC1, 0x44, 0x78, 0x1d10, dataLen, timeout=3000)
        return bytes(res)

    def _standard_cmd_write(self, cmd: int, cmd_param: int = 0, data: bytes = b'\x00', dataLen: int = -1):
        if dataLen == -1:
            dataLen = len(data)

        # switch endianness for cmd_param
        cmd_param = struct.unpack('<I', struct.pack('>I', cmd_param))[0]

        if (dataLen == 0 or data == b'\x00'):
            d = struct.pack("<H", cmd)
            d += struct.pack(">I2x", cmd_param)
            self._dev.ctrl_transfer(0x41, 0x45, 0x78, 0x1d00, d, timeout=3000)
            self._block_until_camera_ready()
            return

        outer_chunk_size = 0x100
        inner_chunk_size = 0x40

        for i in range(0, dataLen, outer_chunk_size):
            outer_chunk = data[i:i+outer_chunk_size]

            initial_data = struct.pack("<H", cmd)
            initial_data += struct.pack(">IH", cmd_param + i, len(outer_chunk))
            self._dev.ctrl_transfer(0x41, 0x45, 0x78, 0x9d00, initial_data, timeout=3000)
            self._block_until_camera_ready()

            for j in range(0, len(outer_chunk), inner_chunk_size):
                inner_chunk = outer_chunk[j:j+inner_chunk_size]
                to_send = len(outer_chunk) - j

                if (to_send <= 8):
                    self._dev.ctrl_transfer(0x41, 0x45, 0x78, 0x1d08 + j, inner_chunk, timeout=3000)
                    self._block_until_camera_ready()
                elif (to_send <= 64):
                    self._dev.ctrl_transfer(0x41, 0x45, 0x78, 0x9d08 + j, inner_chunk[:-8], timeout=3000)
                    self._dev.ctrl_transfer(0x41, 0x45, 0x78, 0x1d08 + j + to_send - 8, inner_chunk[-8:], timeout=3000)
                    self._block_until_camera_ready()
                else:
                    self._dev.ctrl_transfer(0x41, 0x45, 0x78, 0x9d08 + j, inner_chunk, timeout=3000)

    def _standard_cmd_read(self, cmd: int, cmd_param: int = 0, dataLen: int = 0) -> bytes:
        if dataLen == 0:
            return b''

        cmd_param = struct.unpack('<I', struct.pack('>I', cmd_param))[0]

        result = b''
        outer_chunk_size = 0x100
        
        for i in range(0, dataLen, outer_chunk_size):
            to_read = min(dataLen - i, outer_chunk_size)
            initial_data = struct.pack("<H", cmd)
            initial_data += struct.pack(">IH", cmd_param + i, to_read)
            
            self._dev.ctrl_transfer(0x41, 0x45, 0x78, 0x1d00, initial_data, timeout=3000)
            self._block_until_camera_ready()

            res = self._dev.ctrl_transfer(0xC1, 0x44, 0x78, 0x1d08, to_read, timeout=3000)
            result += bytes(res)

        return result

    def pseudo_color_set(self, preview_path: int, color_type: PseudoColorTypes):
        self._standard_cmd_write((CmdCode.pseudo_color | CmdDir.SET), preview_path, struct.pack("<B", color_type))

    def pseudo_color_get(self, preview_path: int = 0) -> PseudoColorTypes:
        res = self._standard_cmd_read(CmdCode.pseudo_color, preview_path, 1)
        if not res: return PseudoColorTypes.PSEUDO_WHITE_HOT
        return PseudoColorTypes(int.from_bytes(res, 'little'))
        
    def shutter_calibration(self):
        # Trigger shutter
        # There isn't a direct "click shutter" in the enum, but 'shutter_vtemp' might be related 
        # or 'sys_reset_to_rom' (calibration).
        # P2Pro repo main.py uses: cam_cmd._standard_cmd_write(P2Pro_CMD.CmdCode.sys_reset_to_rom)
        # Note: sys_reset_to_rom is 0x0805.
        print("[INFO] Triggering Shutter/Calibration...")
        # Try sys_reset_to_rom as a guess for "NUC" (Non-Uniformity Correction)
        self._standard_cmd_write(CmdCode.sys_reset_to_rom)

    def get_device_info(self, dev_info: DeviceInfoType):
        res = self._standard_cmd_read(CmdCode.get_device_info, dev_info, DeviceInfoType_len[dev_info])
        return res
