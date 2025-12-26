import usb.core
import usb.util
import sys

def main():
    print("Searching for T2Pro (VID 0x04b4, PID 0x0100)...")
    # Find device
    dev = usb.core.find(idVendor=0x04b4, idProduct=0x0100)
    
    if dev is None:
        print("Device not found via pyusb.")
        return

    print(f"Device found: {dev}")
    
    print("\n--- Configuration Descriptor ---")
    for cfg in dev:
        print(f"Configuration Value: {cfg.bConfigurationValue}")
        for intf in cfg:
            print(f"  Interface Number: {intf.bInterfaceNumber}, Alt Setting: {intf.bAlternateSetting}")
            print(f"    Class: {intf.bInterfaceClass}, SubClass: {intf.bInterfaceSubClass}, Protocol: {intf.bInterfaceProtocol}")
            
            for ep in intf:
                print(f"    Endpoint Address: {hex(ep.bEndpointAddress)}")
                print(f"      Attributes: {hex(ep.bmAttributes)}")
                print(f"      Max Packet Size: {ep.wMaxPacketSize}")

    # Explicitly check for UVC class (14 = 0x0E)
    # Subclass 1 (Video Control), 2 (Video Streaming)
    print("\n--- Checking for Video Class Interfaces ---")
    uvc_intf = None
    for cfg in dev:
        for intf in cfg:
            if intf.bInterfaceClass == 14:
                print(f"  Found UVC Interface: {intf.bInterfaceNumber} (Subclass {intf.bInterfaceSubClass})")
                
                # Check for Extra Descriptors (Class Specific)
                if intf.extra_descriptors:
                    import binascii
                    import array
                    
                    data = intf.extra_descriptors
                    # Convert to bytes if it's a list or array
                    if isinstance(data, (list, tuple, array.array)):
                        data = bytes(data)
                        
                    print(f"    Extra Descriptors ({len(data)} bytes):")

                    try:
                        print(f"    Hex: {binascii.hexlify(data).decode('utf-8')}")
                        
                        # Simple parsing loop
                        idx = 0
                        while idx < len(data):
                            length = data[idx]
                            if length == 0: break
                            if idx + length > len(data): break
                            
                            chunk = data[idx:idx+length]
                            # Handle short chunks
                            subtype = chunk[2] if length > 2 else -1
                            
                            # VS_FORMAT_UNCOMPRESSED = 4
                            # VS_FRAME_UNCOMPRESSED = 5
                            # VS_FORMAT_MJPEG = 6
                            # VS_FRAME_MJPEG = 7
                            
                            if intf.bInterfaceSubClass == 2: # Streaming
                                if subtype == 4:
                                    fmt_idx = chunk[3]
                                    print(f"      [Format Uncompressed] Index: {fmt_idx}")
                                elif subtype == 5:
                                    frame_idx = chunk[3]
                                    try:
                                        w = chunk[5] + (chunk[6] << 8)
                                        h = chunk[7] + (chunk[8] << 8)
                                        print(f"      [Frame Uncompressed] Index: {frame_idx} | {w}x{h}")
                                    except IndexError:
                                        print(f"      [Frame Uncompressed] Index: {frame_idx} | (Incomplete descriptor)")

                                elif subtype == 6:
                                    fmt_idx = chunk[3]
                                    print(f"      [Format MJPEG] Index: {fmt_idx}")
                                elif subtype == 7:
                                    frame_idx = chunk[3]
                                    try:
                                        w = chunk[5] + (chunk[6] << 8)
                                        h = chunk[7] + (chunk[8] << 8)
                                        print(f"      [Frame MJPEG] Index: {frame_idx} | {w}x{h}")
                                    except IndexError:
                                        print(f"      [Frame MJPEG] Index: {frame_idx} | (Incomplete descriptor)")

                            idx += length
                            
                    except Exception as e:
                         print(f"    Error parsing extras: {e}")

    if dev.is_kernel_driver_active(0):
        print("\nKernel driver is active on Interface 0.")
    else:
        print("\nKernel driver is NOT active on Interface 0.")

if __name__ == "__main__":
    main()
