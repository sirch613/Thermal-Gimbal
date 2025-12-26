import cv2
import time
import sys
import numpy as np

# Try importing the low-level UVC library
try:
    import uvc
    HAS_UVC = True
    print("[INFO] 'uvc' library loaded successfully. Using low-level custom driver.")
except ImportError:
    HAS_UVC = False
    print("[ERROR] 'uvc' library not found. Please run: pip install pupil-labs-uvc")
    print("Falling back to OpenCV standard scanning (which failed previously).")

def scan_uvc_devices():
    if not HAS_UVC:
        return []
    
    print("Scanning USB Video Class devices via libuvc...")
    dev_list = uvc.device_list()
    found = []
    for dev in dev_list:
        try:
            name = dev.name
            vid = dev.idVendor
            pid = dev.idProduct
            print(f"  Found: {name} (VID: {hex(vid)}, PID: {hex(pid)})")
            
            # T2Pro identifiers
            # VID 0x04b4 (Cypress) / PID 0x0100 is common for T2Pro, but let's match broadly
            # Name often contains "T2Pro" or "InfiRay" or "USB Camera" if generic
            
            if vid == 0x04b4 or "T2" in name or "InfiRay" in name:
                print(f"    *** MATCHED T2Pro CANDIDATE: {name} ***")
                found.append(dev)
        except Exception as e:
            print(f"  Error reading device info: {e}")
    return found

def run_driver():
    print("--- InfiRay T2 Pro Deep Driver ---")
    
    devices = scan_uvc_devices()
    
    if not devices:
        print("\n[!] No T2 Pro candidates found via libuvc.")
        print("Please UNPLUG and REPLUG the camera, then press Enter to rescan.")
        input("Press Enter to rescan...")
        devices = scan_uvc_devices()
        
        if not devices:
            print("[!] Still nothing. Falling back to OpenCV check just in case.")
            # Fallback code similar to before could go here, but let's stick to UVC goal
            return

    # Pick the first candidate
    dev = devices[0]
    print(f"\nAttempting to open: {dev.name}")
    
    try:
        cap = uvc.Capture(dev.uid)
        
        print("Available modes:")
        best_mode = None
        for mode in cap.modes:
            print(f"  {mode}")
            # Look for 256x192
            if mode.width == 256 and mode.height == 192:
                best_mode = mode
            # Fallback to similar
            if not best_mode and mode.width == 256: 
                best_mode = mode
        
        if not best_mode:
            print("[WARN] No exact 256x192 mode found. Using first available.")
            best_mode = cap.modes[0]
            
        print(f"Selected Mode: {best_mode}")
        cap.frame_mode = best_mode
        
        print("Opening Stream...")
        
        cv2.namedWindow("T2 Pro Deep Feed", cv2.WINDOW_NORMAL)
        
        while True:
            # Capture using UVC
            frame = cap.get_frame_robust()
            
            # frame is a uvc.Frame object
            # It has .gray (numpy array) or .bgr
            
            # The T2 Pro sends 16-bit data usually? Or YUYV?
            # uvc often handles conversion.
            
            if hasattr(frame, 'gray'):
                img = frame.gray
            elif hasattr(frame, 'bgr'):
                img = frame.img # bgr
            else:
                img = frame.img # generic
                
            # If it's 16-bit (thermal raw), we might need to normalize it to see anything
            if img.dtype == np.uint16:
                # Normalize 16-bit to 8-bit for display
                # Simple min-max
                img_float = img.astype(np.float32)
                min_val = np.min(img_float)
                max_val = np.max(img_float)
                img_norm = ((img_float - min_val) / (max_val - min_val + 1e-6) * 255).astype(np.uint8)
                img_display = cv2.applyColorMap(img_norm, cv2.COLORMAP_INFERNO)
            else:
                img_display = img
                
            cv2.putText(img_display, f"{dev.name} {img.shape}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow("T2 Pro Deep Feed", img_display)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        cap.close()
        cv2.destroyAllWindows()
        
    except Exception as e:
        print(f"[ERROR] Failed to open/stream device: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    if not HAS_UVC:
        sys.exit(1)
    run_driver()
