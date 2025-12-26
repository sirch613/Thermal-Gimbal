import ctypes
import ctypes.util
import time
import sys
import os
import numpy as np
import cv2
import serial
import serial.tools.list_ports
import threading
from thermal_ai import ThermalAI
import t2pro_cmd

# --- MOTOR CONTROL SIDEBAR CONSTANTS ---
SIDEBAR_WIDTH = 200
VIDEO_WIDTH = 1024
VIDEO_HEIGHT = 768
TOTAL_WIDTH = VIDEO_WIDTH + SIDEBAR_WIDTH

# Colors (Dieter Rams / Braun Inspired)
C_BG = (227, 227, 227)      # #E3E3E3 Matte Light Grey
C_TEXT = (40, 40, 40)       # #282828 Almost Black
C_BTN_OFF = (60, 60, 60)    # #3C3C3C Dark Grey (Braun Button)
C_BTN_HOVER = (90, 90, 90)  # Lighter Grey
C_BTN_ACTIVE = (0, 69, 255) # #FF4500 Orange (BGR)
C_BTN_BORDER = (40, 40, 40) # Dark Border
C_BTN_TEXT = (230, 230, 230)# Off White Text

import queue

class SerialManager:
    def __init__(self):
        self.ser = None
        self.connected = False
        self.port_name = "Scanning..."
        self.lock = threading.Lock()
        self.running = True
        
        # Async Command Queue
        self.cmd_queue = queue.Queue()
        self.writer_thread = threading.Thread(target=self._async_writer_loop, daemon=True)
        self.writer_thread.start()
        
        # Position Tracking (Open Loop)
        self.pan_steps = 0
        self.tilt_steps = 0
        
        # Limit Logic Disabled for Debugging
        self.MAX_PAN_STEPS = 999999
        self.MAX_TILT_STEPS = 999999
        
        self.t = threading.Thread(target=self._auto_connect_loop, daemon=True)
        self.t.start()

    def _async_writer_loop(self):
        """Background thread to drain the command queue."""
        while self.running:
            try:
                cmd = self.cmd_queue.get(timeout=0.1)
                if self.connected and self.ser:
                     try:
                        self.ser.write(f"{cmd}\n".encode())
                     except Exception as e:
                        print(f"[SERIAL] Write Error: {e}")
                        self.connected = False
                self.cmd_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[SERIAL] Writer Thread Error: {e}")
        
    def _auto_connect_loop(self):
        while self.running:
            if not self.connected:
                ports = list(serial.tools.list_ports.comports())
                for p in ports:
                    if "16C0" in p.hwid or "usbmodem" in p.device:
                        try:
                            self.ser = serial.Serial(p.device, 115200, timeout=1)
                            self.port_name = p.device
                            self.connected = True
                            print(f"[SERIAL] Connected to {p.device}")
                            time.sleep(2) 
                            # SAFETY: Stop all motors on connect
                            self.send_command("J1:0")
                            self.send_command("J2:0")
                            self.send_command("J3:0")
                            self.send_command("J4:0")
                            break
                        except Exception:
                            pass
            else:
                try:
                    if self.ser.in_waiting:
                        self.ser.read(self.ser.in_waiting)
                except Exception:
                    self.connected = False
                    self.ser = None
            time.sleep(2)
            
    def send_command(self, cmd):
        if self.connected:
             self.cmd_queue.put(cmd)
                
    def stop(self):
        self.running = False
        
class Button:
    def __init__(self, x, y, w, h, text, cmd, is_orange=False, is_toggle=False):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.text = text
        self.cmd = cmd
        self.is_orange = is_orange
        self.is_toggle = is_toggle
        self.active = False 
        self.hover = False
        self.clicked = False 
        self.disabled = False 
        
    def draw(self, img):
        is_on = self.active if self.is_toggle else self.clicked
        
        if self.disabled:
            fill = (100, 100, 100) 
            txt_color = (150, 150, 150)
        elif is_on:
            fill = C_BTN_ACTIVE
            txt_color = (255, 255, 255)
        elif self.hover:
            fill = C_BTN_HOVER
            txt_color = (255, 255, 255)
        else:
            fill = C_BTN_OFF
            txt_color = C_BTN_TEXT
            
        cx, cy = self.x + self.w//2, self.y + self.h//2
        radius = min(self.w, self.h) // 2
        cv2.circle(img, (cx, cy), radius, fill, -1, cv2.LINE_AA)
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.45
        thick = 1
        tsize, base = cv2.getTextSize(self.text, font, scale, thick)
        tx = cx - tsize[0]//2
        ty = cy + tsize[1]//2
        cv2.putText(img, self.text, (tx, ty), font, scale, txt_color, thick, cv2.LINE_AA)

    def is_inside(self, ix, iy):
        dx = ix - (self.x + self.w//2)
        dy = iy - (self.y + self.h//2)
        radius = min(self.w, self.h) // 2
        return (dx*dx + dy*dy) <= radius*radius

# Load libuvc
LIBUVC_PATH = "/opt/homebrew/lib/libuvc.dylib"
try:
    libuvc = ctypes.CDLL(LIBUVC_PATH)
except OSError:
    try:
        path = ctypes.util.find_library("uvc")
        if path:
            libuvc = ctypes.CDLL(path)
        else:
            print("[ERROR] Could not load libuvc.dylib.")
            sys.exit(1)
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

uvc_context_p = ctypes.c_void_p
uvc_device_p = ctypes.c_void_p
uvc_device_handle_p = ctypes.c_void_p
uvc_stream_ctrl_p = ctypes.c_void_p

class uvc_stream_ctrl(ctypes.Structure):
    _fields_ = [("data", ctypes.c_char * 1024)] 

class UVCFrame(ctypes.Structure):
    _fields_ = [
        ("data", ctypes.POINTER(ctypes.c_uint8)),
        ("data_bytes", ctypes.c_size_t),
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("frame_format", ctypes.c_int),
        ("step", ctypes.c_size_t),
        ("sequence", ctypes.c_uint32),
        ("capture_time", ctypes.c_void_p), 
        ("source", ctypes.c_void_p), 
        ("library_owns_data", ctypes.c_uint8)
    ]

FRAME_CALLBACK_TYPE = ctypes.CFUNCTYPE(None, ctypes.POINTER(UVCFrame), ctypes.c_void_p)

libuvc.uvc_init.argtypes = [ctypes.POINTER(uvc_context_p), ctypes.c_void_p]
libuvc.uvc_init.restype = ctypes.c_int
libuvc.uvc_exit.argtypes = [uvc_context_p]
libuvc.uvc_exit.restype = None
libuvc.uvc_get_device_list.argtypes = [uvc_context_p, ctypes.POINTER(ctypes.POINTER(uvc_device_p))]
libuvc.uvc_get_device_list.restype = ctypes.c_int
libuvc.uvc_free_device_list.argtypes = [ctypes.POINTER(uvc_device_p), ctypes.c_uint8]
libuvc.uvc_free_device_list.restype = None
libuvc.uvc_find_device.argtypes = [uvc_context_p, ctypes.POINTER(uvc_device_p), ctypes.c_int, ctypes.c_int, ctypes.c_char_p]
libuvc.uvc_find_device.restype = ctypes.c_int
libuvc.uvc_open.argtypes = [uvc_device_p, ctypes.POINTER(uvc_device_handle_p)]
libuvc.uvc_open.restype = ctypes.c_int
libuvc.uvc_close.argtypes = [uvc_device_handle_p]
libuvc.uvc_close.restype = None
libuvc.uvc_get_stream_ctrl_format_size.argtypes = [uvc_device_handle_p, ctypes.POINTER(uvc_stream_ctrl), ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
libuvc.uvc_get_stream_ctrl_format_size.restype = ctypes.c_int
libuvc.uvc_start_streaming.argtypes = [uvc_device_handle_p, ctypes.POINTER(uvc_stream_ctrl), FRAME_CALLBACK_TYPE, ctypes.c_void_p, ctypes.c_uint8]
libuvc.uvc_start_streaming.restype = ctypes.c_int
libuvc.uvc_stop_streaming.argtypes = [uvc_device_handle_p]
libuvc.uvc_stop_streaming.restype = None

latest_frame = None

def frame_cb(frame_ptr, user_ptr):
    global latest_frame
    if not frame_ptr: return
    frame = frame_ptr.contents
    if frame.data_bytes > 0:
        try:
            raw_data = ctypes.string_at(frame.data, frame.data_bytes)
            latest_frame = (raw_data, frame.width, frame.height)
        except: pass

c_frame_cb = FRAME_CALLBACK_TYPE(frame_cb)

def main():
    global latest_frame
    if os.path.exists(".nuc_trigger"):
        print("[LAUNCH] performing Shutter Calibration...")
        try:
            with t2pro_cmd.T2ProCmd() as cmd:
                cmd.shutter_calibration()
            print("[LAUNCH] NUC Complete.")
        except Exception as e: print(f"[LAUNCH] NUC Failed: {e}")
        finally:
            try: os.remove(".nuc_trigger")
            except: pass
        time.sleep(1.0)
    
    ctx = uvc_context_p()
    res = libuvc.uvc_init(ctypes.byref(ctx), None)
    if res < 0: return

    dev = uvc_device_p()
    device_found = False
    res = libuvc.uvc_find_device(ctx, ctypes.byref(dev), 0x04b4, 0x0100, None)
    if res < 0: res = libuvc.uvc_find_device(ctx, ctypes.byref(dev), 0, 0, None)
    
    device_found = (res >= 0)
    devh = None
    if device_found:
        devh = uvc_device_handle_p()
        if libuvc.uvc_open(dev, ctypes.byref(devh)) < 0:
            device_found = False
            devh = None

    negotiated = False
    if device_found:
        ctrl = uvc_stream_ctrl()
        for w, h, fps, fmt in [(256, 196, 0, 1), (256, 196, 9, 1), (256, 196, 25, 1)]:
            if libuvc.uvc_get_stream_ctrl_format_size(devh, ctypes.byref(ctrl), fmt, w, h, fps) >= 0:
                negotiated = True; break
        if negotiated:
            if libuvc.uvc_start_streaming(devh, ctypes.byref(ctrl), c_frame_cb, None, 0) < 0:
                negotiated = False

    # --- UI INITIALIZATION ---
    colormap_idx = 4
    colormaps = [cv2.COLORMAP_INFERNO, cv2.COLORMAP_JET, cv2.COLORMAP_TURBO, cv2.COLORMAP_OCEAN, cv2.COLORMAP_BONE]
    colormap_names = ["Inferno", "Jet", "Turbo", "Ocean", "White Hot", "Black Hot"]
    use_blur, use_clahe, use_sharpen, zoom_factor = True, False, False, 1.0
    use_matrix, use_focus, matrix_acc = False, False, None
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    sharpen_kernel = np.array([[-1,-1,-1], [-1,9,-1], [-1,-1,-1]])
    
    serial_mgr = SerialManager()
    thermal_ai = ThermalAI()
    buttons = []
    bx_center = VIDEO_WIDTH + SIDEBAR_WIDTH // 2
    btn_sz, spacing, cur_y = 40, 10, 15
    
    btn_gear = Button(VIDEO_WIDTH + SIDEBAR_WIDTH - 30, 15, 20, 20, "*", "SETTINGS", is_toggle=True); buttons.append(btn_gear)
    cur_y += 30
    # Tracker
    buttons.append(Button(bx_center-btn_sz-spacing//2, cur_y, btn_sz, btn_sz, "<", "J3:-6000"))
    buttons.append(Button(bx_center+spacing//2, cur_y, btn_sz, btn_sz, ">", "J3:6000"))
    cur_y += btn_sz+spacing
    buttons.append(Button(bx_center-btn_sz-spacing//2, cur_y, btn_sz, btn_sz, "Up", "J4:4000"))
    buttons.append(Button(bx_center+spacing//2, cur_y, btn_sz, btn_sz, "Dn", "J4:-4000"))
    cur_y += btn_sz+30
    # Shooter
    buttons.append(Button(bx_center-btn_sz-spacing//2, cur_y, btn_sz, btn_sz, "<", "J1:-6000"))
    buttons.append(Button(bx_center+spacing//2, cur_y, btn_sz, btn_sz, ">", "J1:6000"))
    cur_y += btn_sz+spacing
    buttons.append(Button(bx_center-btn_sz-spacing//2, cur_y, btn_sz, btn_sz, "Up", "J2:4000"))
    buttons.append(Button(bx_center+spacing//2, cur_y, btn_sz, btn_sz, "Dn", "J2:-4000"))
    cur_y += btn_sz+30
    
    btn_auto = Button(bx_center-25, cur_y, 50, 50, "AUTO", "AUTO_TRACK", is_toggle=True); buttons.append(btn_auto); cur_y += 50+spacing
    btn_lock = Button(bx_center-25, cur_y, 50, 50, "LOCK", "PAN_LOCK", is_toggle=True); buttons.append(btn_lock); cur_y += 50+spacing
    btn_ai = Button(bx_center-25, cur_y, 50, 50, "AI", "AI_TRACK", is_toggle=True); buttons.append(btn_ai); cur_y += 50+spacing
    btn_mode = Button(bx_center-25, cur_y, 50, 50, "MODE", "AI_MODE", is_toggle=False); buttons.append(btn_mode); cur_y += 50+spacing
    btn_set = Button(bx_center-25, cur_y, 50, 50, "SET", "SETTINGS", is_toggle=True); buttons.append(btn_set)

    held_button, last_hold_time, last_track_time = None, 0, 0
    frame_count, last_time, last_frame_received_time = 0, time.time(), time.time()
    TRACK_INTERVAL, DEADBAND, actual_fps = 0.016, 15, 0.0

    def mouse_event(event, x, y, flags, param):
        nonlocal held_button, last_hold_time
        if event == cv2.EVENT_LBUTTONDOWN:
            for b in buttons:
                if b.is_inside(x, y) and not b.disabled:
                    if b.is_toggle: b.active = not b.active
                    elif b.cmd == "AI_MODE": thermal_ai.set_mode("PERSON" if thermal_ai.mode == "DRONE" else "DRONE")
                    else:
                        is_tilt = any(k in b.cmd for k in ["J2", "J4", "M2", "M4"])
                        if not(is_tilt and btn_lock.active):
                            b.clicked = True; held_button = b; serial_mgr.send_command(b.cmd); last_hold_time = time.time()
        elif event == cv2.EVENT_LBUTTONUP:
            for i in [1,2,3,4]: serial_mgr.send_command(f"J{i}:0")
            held_button = None
            for b in buttons: 
                if not b.is_toggle: b.clicked = False
        elif event == cv2.EVENT_MOUSEMOVE:
            for b in buttons: b.hover = b.is_inside(x, y)

    cv2.namedWindow("T2Pro Enhanced", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("T2Pro Enhanced", mouse_event)

    try:
        while True:
            curr_frame = latest_frame
            processed = False
            if not device_found:
                thermal_data = np.zeros((192, 256), dtype=np.uint16)
                last_frame_received_time = time.time(); processed = True; time.sleep(0.01)
            elif curr_frame:
                latest_frame = None; last_frame_received_time = time.time()
                data, w, h = curr_frame
                arr = np.frombuffer(data, dtype=np.uint16)
                if len(arr) >= w*h:
                    thermal_data = arr.reshape((h, w))[0:192, :]; processed = True

            if processed:
                if zoom_factor > 1.0:
                    h_r, w_r = thermal_data.shape
                    cx, cy = w_r//2, h_r//2
                    cw, ch = int(w_r/zoom_factor), int(h_r/zoom_factor)
                    x1, y1 = max(0, cx-cw//2), max(0, cy-ch//2)
                    thermal_data = cv2.resize(thermal_data[y1:y1+ch, x1:x1+cw], (w_r, h_r))

                d_min, d_max = np.min(thermal_data), np.max(thermal_data)
                minVal, maxVal, minLoc, maxLoc = cv2.minMaxLoc(thermal_data)
                spot_raw = np.mean(thermal_data[94:99, 126:131])

                frame_count += 1
                if time.time() - last_time > 1.0:
                    actual_fps = frame_count / (time.time() - last_time)
                    frame_count, last_time = 0, time.time()

                vmin, vmax = np.percentile(thermal_data, 2), np.percentile(thermal_data, 98)
                if vmax <= vmin: vmin, vmax = d_min, d_max + 1
                norm = np.clip(thermal_data, vmin, vmax)
                norm = ((norm - vmin) / (vmax - vmin) * 255.0).astype(np.uint8)

                ai_detections = thermal_ai.process_frame(thermal_data)[1] if btn_ai.active else []
                if use_matrix:
                    if matrix_acc is None: matrix_acc = norm.astype(np.float32)
                    else: cv2.accumulateWeighted(norm, matrix_acc, 0.5); norm = cv2.convertScaleAbs(matrix_acc)
                else: matrix_acc = None

                if use_blur: norm = cv2.medianBlur(norm, 3)
                if use_clahe: norm = clahe.apply(norm)
                if use_sharpen: norm = cv2.filter2D(norm, -1, sharpen_kernel)
                
                if colormap_idx < 4: color_img = cv2.applyColorMap(norm, colormaps[colormap_idx])
                elif colormap_idx == 4: color_img = cv2.cvtColor(norm, cv2.COLOR_GRAY2BGR)
                else: color_img = cv2.cvtColor(cv2.bitwise_not(norm), cv2.COLOR_GRAY2BGR)

                upscale = cv2.resize(color_img, (1024, 768), cv2.INTER_CUBIC)
                if btn_ai.active: thermal_ai.draw_detections(upscale, ai_detections, 4.0)
                
                status_txt = f"Zoom: {zoom_factor:.1f}x {'MATRIX ' if use_matrix else ''}{'AI '+thermal_ai.mode if btn_ai.active else ''}"
                cv2.putText(upscale, status_txt, (10, 750), 0, 0.6, (0, 255, 255), 2)
                cv2.line(upscale, (512, 364), (512, 404), (200, 200, 200), 1)
                cv2.line(upscale, (492, 384), (532, 384), (200, 200, 200), 1)

                if btn_auto.active:
                    tx, ty = None, None
                    if btn_ai.active and ai_detections:
                        b = ai_detections[0]['box']; tx, ty = (b[0]+b[2])/2, (b[1]+b[3])/2
                    elif spot_raw > 0: tx, ty = maxLoc
                    if tx is not None and time.time() - last_track_time > TRACK_INTERVAL:
                        ex, ey = tx - 128, ty - 96
                        if abs(ex) > DEADBAND: serial_mgr.send_command(f"J3:{int(np.clip(ex*60, -8000, 8000))}")
                        else: serial_mgr.send_command("J3:0")
                        if not btn_lock.active:
                            if abs(ey) > DEADBAND: serial_mgr.send_command(f"J4:{int(np.clip(-ey*60, -6000, 6000))}")
                            else: serial_mgr.send_command("J4:0")
                        last_track_time = time.time()

                sidebar = np.zeros((768, 200, 3), dtype=np.uint8); sidebar[:] = C_BG
                def draw_lbl(t, y, s=0.5, c=C_TEXT, th=1):
                    sz = cv2.getTextSize(t, 0, s, th)[0]; cv2.putText(sidebar, t, ((200-sz[0])//2, y), 0, s, c, th)
                draw_lbl("CONTROL", 25, 0.6); cv2.circle(sidebar, (20, 20), 4, (0,200,0) if serial_mgr.connected else (0,0,200), -1)
                cv2.line(sidebar, (20, 35), (180, 35), (200,200,200), 1)
                draw_lbl("TRACKER", 55, 0.4, (100,100,100)); draw_lbl("SHOOTER", 185, 0.4, (100,100,100)); draw_lbl("AUTOMATION", 315, 0.4, (100,100,100))
                if not serial_mgr.connected: draw_lbl("TEENSY", 550, 0.5, (0,0,255), 2); draw_lbl("DISCONNECTED", 575, 0.5, (0,0,255), 2)
                for b in buttons: b.disabled = (not serial_mgr.connected and b.cmd.startswith(("J", "M")))
                draw_lbl("version 342", 750, 0.4, (150,150,150))
                
                combined = np.hstack((upscale, sidebar))
                for b in buttons: b.draw(combined)
                if btn_set.active or btn_gear.active:
                    cv2.rectangle(combined, (50,50), (974,718), (30,30,30), -1)
                    cv2.putText(combined, "SYSTEM SETTINGS", (80, 100), 0, 0.8, C_BTN_ACTIVE, 2)
                    cv2.putText(combined, f"Camera: {'Found' if device_found else 'Offline'}", (80, 160), 0, 0.6, (255,255,255), 1)
                cv2.imshow("T2Pro Enhanced", combined)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): break
            elif key == ord('c'): colormap_idx = (colormap_idx + 1) % 6
            elif key == ord('b'): use_blur = not use_blur
            elif key == ord('n'): thermal_ai.set_mode("PERSON" if thermal_ai.mode == "DRONE" else "DRONE")
            elif key == ord('r'):
                if device_found:
                    with open(".nuc_trigger", "w") as f: f.write("1")
                    break
            elif key == ord('g'):
                if device_found:
                    try:
                        with t2pro_cmd.T2ProCmd() as c:
                            print(f"Serial: {c.get_device_info(t2pro_cmd.DeviceInfoType.DEV_INFO_GET_SN).hex()}")
                    except: pass
            elif key == ord('='): zoom_factor = min(15.0, zoom_factor + 0.5)
            elif key == ord('-'): zoom_factor = max(1.0, zoom_factor - 0.5)

            if device_found and time.time() - last_frame_received_time > 5.0: break

    finally:
        serial_mgr.stop()
        if device_found and devh: libuvc.uvc_stop_streaming(devh)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
