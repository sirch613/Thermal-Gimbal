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
import queue
from thermal_ai import ThermalAI
import t2pro_cmd

# --- MOTOR CONTROL SIDEBAR CONSTANTS ---
# --- MOTOR CONTROL SIDEBAR CONSTANTS ---
SIDEBAR_WIDTH = 200
VIDEO_WIDTH = 1024
VIDEO_HEIGHT = 768
TOTAL_WIDTH = VIDEO_WIDTH + SIDEBAR_WIDTH

# Colors (Dieter Rams / Braun Inspired)
# Palette: Matte Light Grey BG, Dark Grey/Black Buttons, Orange Accents
C_BG = (227, 227, 227)      # #E3E3E3 Matte Light Grey
C_TEXT = (40, 40, 40)       # #282828 Almost Black
C_BTN_OFF = (60, 60, 60)    # #3C3C3C Dark Grey (Braun Button)
C_BTN_HOVER = (90, 90, 90)  # Lighter Grey
C_BTN_ACTIVE = (0, 69, 255) # #FF4500 Orange (BGR)
C_BTN_BORDER = (40, 40, 40) # Dark Border
C_BTN_TEXT = (230, 230, 230)# Off White Text

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

    def send_command(self, cmd):
        """Put a command on the queue for asynchronous sending."""
        if self.connected:
            self.cmd_queue.put(cmd)
        
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
                            time.sleep(2) # Wait for DTR reset if any
                            # SAFETY: Stop all motors on connect (in case they were left running)
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
        if self.connected and self.ser:
            # Limits disabled for hardware testing
            pass

            try:
                with self.lock:
                    self.ser.write(f"{cmd}\n".encode())
                print(f"[SERIAL] Sent: {cmd} | PanPos: {self.pan_steps}")
            except Exception:
                self.connected = False
                
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
        self.active = False # For toggles
        self.hover = False
        self.clicked = False # Momentary state
        self.disabled = False # Connection safety state
        
    def draw(self, img):
        # Color Logic
        is_on = self.active if self.is_toggle else self.clicked
        
        if self.disabled:
            fill = (100, 100, 100) # Neutral Grey
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
        
        # Anti-aliased Circle
        cv2.circle(img, (cx, cy), radius, fill, -1, cv2.LINE_AA)
        
        # Text
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
        return (dx*dx + dy*dy) <= radius*radius # Circular Hit Test



# Load libuvc via ctypes
# Homebrew path usually: /opt/homebrew/lib/libuvc.dylib
LIBUVC_PATH = "/opt/homebrew/lib/libuvc.dylib"

try:
    libuvc = ctypes.CDLL(LIBUVC_PATH)
    print(f"[INFO] Loaded libuvc from {LIBUVC_PATH}")
except OSError:
    try:
        # Try finding it automatically
        path = ctypes.util.find_library("uvc")
        if path:
            libuvc = ctypes.CDLL(path)
            print(f"[INFO] Loaded libuvc from {path}")
        else:
            print("[ERROR] Could not load libuvc.dylib. Make sure 'brew install libuvc' is done.")
            sys.exit(1)
    except Exception as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

# Definitions must come before usage
uvc_context_p = ctypes.c_void_p
uvc_device_p = ctypes.c_void_p
uvc_device_handle_p = ctypes.c_void_p
uvc_stream_ctrl_p = ctypes.c_void_p

# Data Structures
class uvc_stream_ctrl(ctypes.Structure):
    _fields_ = [("data", ctypes.c_char * 1024)] 

# We must define the FULL UVCFrame struct before creating the callback type
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
        ("source", ctypes.c_void_p), # uvc_device_handle_t*
        ("library_owns_data", ctypes.c_uint8)
    ]

# Frame Callback Type
FRAME_CALLBACK_TYPE = ctypes.CFUNCTYPE(None, ctypes.POINTER(UVCFrame), ctypes.c_void_p)

# Function signatures
libuvc.uvc_init.argtypes = [ctypes.POINTER(uvc_context_p), ctypes.c_void_p]
libuvc.uvc_init.restype = ctypes.c_int

libuvc.uvc_exit.argtypes = [uvc_context_p]
libuvc.uvc_exit.restype = None

libuvc.uvc_get_device_list.argtypes = [uvc_context_p, ctypes.POINTER(ctypes.POINTER(uvc_device_p))]
libuvc.uvc_get_device_list.restype = ctypes.c_int

libuvc.uvc_free_device_list.argtypes = [ctypes.POINTER(uvc_device_p), ctypes.c_uint8]
libuvc.uvc_free_device_list.restype = None

libuvc.uvc_get_bus_number.argtypes = [uvc_device_p]
libuvc.uvc_get_bus_number.restype = ctypes.c_uint8

libuvc.uvc_get_device_address.argtypes = [uvc_device_p]
libuvc.uvc_get_device_address.restype = ctypes.c_uint8

libuvc.uvc_find_device.argtypes = [uvc_context_p, ctypes.POINTER(uvc_device_p), ctypes.c_int, ctypes.c_int, ctypes.c_char_p]
libuvc.uvc_find_device.restype = ctypes.c_int

libuvc.uvc_open.argtypes = [uvc_device_p, ctypes.POINTER(uvc_device_handle_p)]
libuvc.uvc_open.restype = ctypes.c_int

libuvc.uvc_close.argtypes = [uvc_device_handle_p]
libuvc.uvc_close.restype = None

libuvc.uvc_get_stream_ctrl_format_size.argtypes = [uvc_device_handle_p, ctypes.POINTER(uvc_stream_ctrl), ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
libuvc.uvc_get_stream_ctrl_format_size.restype = ctypes.c_int

# HERE IS THE KEY FIX: Use FRAME_CALLBACK_TYPE in argtypes
libuvc.uvc_start_streaming.argtypes = [uvc_device_handle_p, ctypes.POINTER(uvc_stream_ctrl), FRAME_CALLBACK_TYPE, ctypes.c_void_p, ctypes.c_uint8]
libuvc.uvc_start_streaming.restype = ctypes.c_int

libuvc.uvc_stop_streaming.argtypes = [uvc_device_handle_p]
libuvc.uvc_stop_streaming.restype = None

# Global latest frame
latest_frame = None

def frame_cb(frame_ptr, user_ptr):
    global latest_frame
    if not frame_ptr:
        return
        
    f = frame_ptr.contents
    if f.data_bytes > 0:
        try:
            # COPY the data immediately to avoid pointer invalidation
            # Using bytearray ensures a new memory block is allocated
            raw_data = bytearray(ctypes.string_at(f.data, f.data_bytes))
            
            # Simple atomic update
            latest_frame = (raw_data, f.width, f.height)
            
            # Debug: Log occasionally
            if not hasattr(frame_cb, "count"): frame_cb.count = 0
            frame_cb.count += 1
            if frame_cb.count <= 5 or frame_cb.count % 500 == 0:
                print(f"[CALLBACK] Received frame #{frame_cb.count}, {f.width}x{f.height}")
        except Exception as e:
            print(f"[CALLBACK ERROR] {e}")

# Create the callback instance
c_frame_cb = FRAME_CALLBACK_TYPE(frame_cb)

# Descriptor Structures
class uvc_format_desc(ctypes.Structure):
    pass

class uvc_frame_desc(ctypes.Structure):
    pass

uvc_format_desc._fields_ = [
    ("next", ctypes.POINTER(uvc_format_desc)),
    ("bDescriptorSubtype", ctypes.c_int), # enum
    ("bFormatIndex", ctypes.c_uint8),
    ("bNumFrameDescriptors", ctypes.c_uint8),
    ("guidFormat", ctypes.c_uint8 * 16), # union in C, simplified here
    ("frame_descs", ctypes.POINTER(uvc_frame_desc))
]

uvc_frame_desc._fields_ = [
    ("next", ctypes.POINTER(uvc_frame_desc)),
    ("bDescriptorSubtype", ctypes.c_int),
    ("bFrameIndex", ctypes.c_uint8),
    ("bmCapabilities", ctypes.c_uint8),
    ("wWidth", ctypes.c_uint16),
    ("wHeight", ctypes.c_uint16),
    ("dwMinBitRate", ctypes.c_uint32),
    ("dwMaxBitRate", ctypes.c_uint32),
    ("dwMaxVideoFrameBufferSize", ctypes.c_uint32),
    ("dwDefaultFrameInterval", ctypes.c_uint32),
    ("dwMinFrameInterval", ctypes.c_uint32),
    ("dwMaxFrameInterval", ctypes.c_uint32),
    ("dwFrameIntervalStep", ctypes.c_uint32),
    # flexible array member follows, irrelevant for pointer access
]

libuvc.uvc_get_format_descs.argtypes = [uvc_device_handle_p]
libuvc.uvc_get_format_descs.restype = ctypes.POINTER(uvc_format_desc)

def dump_descriptors(devh):
    print("--- Supported Formats ---")
    fmt = libuvc.uvc_get_format_descs(devh)
    
    first_fmt_idx = None
    first_w = 0
    first_h = 0
    first_fps = 0
    
    while fmt:
        f = fmt.contents
        print(f"Format Index: {f.bFormatIndex} | Subtype: {f.bDescriptorSubtype} | Num Frames: {f.bNumFrameDescriptors}")
        
        # Iterate frames
        frame = f.frame_descs
        while frame:
            fr = frame.contents
            w = fr.wWidth
            h = fr.wHeight
            idx = fr.bFrameIndex
            
            # FPS = 10000000 / interval
            fps = 0
            if fr.dwDefaultFrameInterval > 0:
                fps = int(10000000 / fr.dwDefaultFrameInterval)
                
            print(f"  Frame Index: {idx} | Res: {w}x{h} | FPS: {fps}")
            
            if first_fmt_idx is None:
                first_fmt_idx = f.bFormatIndex
                first_w = w
                first_h = h
                first_fps = fps
                
            frame = fr.next
            
        fmt = f.next
    print("-------------------------")
    return first_fmt_idx, first_w, first_h, first_fps



def main():
    global latest_frame
    ctx = uvc_context_p()
    res = libuvc.uvc_init(ctypes.byref(ctx), None)
    if res < 0:
        print("uvc_init failed")
        return

    print("Scanning for T2Pro (VID 0x04b4)...")
    dev = uvc_device_p()
    
    # Try finding specifically T2Pro
    res = libuvc.uvc_find_device(ctx, ctypes.byref(dev), 0x04b4, 0x0100, None)
    
    if res < 0:
        print("T2Pro not found explicitly. Searching generic (0,0,0)...")
        res = libuvc.uvc_find_device(ctx, ctypes.byref(dev), 0, 0, None)
        if res < 0:
             print("No UVC devices found via find_device.")
             return

    print("Device found. Opening...")
    devh = uvc_device_handle_p()
    res = libuvc.uvc_open(dev, ctypes.byref(devh))
    if res < 0:
        print(f"Could not open device. Result: {res}")
        return

    print("Negotiating Stream...")
    ctrl = uvc_stream_ctrl()
    
    negotiated = False
    
    # Priority list
    # Stabilizing: Let device pick via '0' (Any) first.
    combinations = [
        (256, 196, 0, 1),  # Try 'Any' FPS (Let FW decide)
        (256, 196, 9, 1),
        (256, 196, 25, 1), 
    ]
    
    for w, h, fps, fmt in combinations:
        print(f"Trying {w}x{h} @ {fps}fps (Format Index {fmt})...")
        res = libuvc.uvc_get_stream_ctrl_format_size(devh, ctypes.byref(ctrl), fmt, w, h, fps)
        if res >= 0:
            print(f"[SUCCESS] Negotiated {w}x{h} @ {fps}fps!")
            negotiated = True
            break
            
    if not negotiated:
        print("Explicit negotiation failed. Trying 'Any' (0,0,0,0)...")
        res = libuvc.uvc_get_stream_ctrl_format_size(devh, ctypes.byref(ctrl), 0, 0, 0, 0)
        if res < 0:
            print(f"FATAL: Could not negotiate ANY stream format. Error: {res}")
            return # Exit if negotiation fails
        else:
            print("[SUCCESS] Auto-negotiated generic format.")
            negotiated = True

    if negotiated:
        print("Starting Stream...")
        res = libuvc.uvc_start_streaming(devh, ctypes.byref(ctrl), c_frame_cb, None, 0)
        
        if res < 0:
            print(f"Start streaming failed: {res}")
        else:
            print("Stream started!")
            print("Controls:")
            print("  q: Quit")
            print("  c: Cycle Colormap")
            print("  b: Toggle Blur (Deresolve Noise)")
            print("  p: Cycle HW Pseudo-Color")
            print("  g: Get Device Info")
            print("  r: Shutter Calibrate (NUC)")
            
            # Interactive State
            # Default to 4 (Grayscale/White Hot) per user request
            colormap_idx = 4
            colormaps = [cv2.COLORMAP_INFERNO, cv2.COLORMAP_JET, cv2.COLORMAP_TURBO, cv2.COLORMAP_OCEAN, cv2.COLORMAP_BONE]
            colormap_names = ["Inferno", "Jet", "Turbo", "Ocean", "White Hot", "Black Hot"]
            use_blur = True
            use_clahe = False
            use_sharpen = False
            
            # Commercial Features
            zoom_factor = 1.0
            use_matrix = False # Temporal Smoothing
            use_tracking = False # Hot Spot Tracking
            use_focus = False # Focus Assist
            
            # NUC Trigger (Automatic on start)
            # NUC Trigger (Removed per user request - Manual only)
            # try:
            #     if use_t2pro_cmd:
            #         t2pro_cmd.shutter_calibration()
            # except Exception as e:
            #     print(f"[WARN] Failed to trigger initial NUC: {e}")
            
            print("[MAIN] Loading ThermalAI (Drone Detection)...")
            thermal_ai = ThermalAI()
            print("[MAIN] ThermalAI ready.")
            
            # Create Sidebar Buttons (Compact Layout)
            serial_mgr = SerialManager()
            buttons = []
            
            # Layout Helper
            bx_center = VIDEO_WIDTH + SIDEBAR_WIDTH // 2
            btn_sz = 40 # Smaller buttons (40px dia)
            spacing = 10
            
            # Y-Cursor (Tightened for no top gap)
            cur_y = 15
            
            # --- GEAR BUTTON (SETTINGS) ---
            # Place at top right of sidebar area
            btn_settings = Button(VIDEO_WIDTH + SIDEBAR_WIDTH - 30, 15, 20, 20, "*", "SETTINGS", is_toggle=True)
            buttons.append(btn_settings)
            
            # --- TRACKER GROUP ---
            # Header drawn in loop
            cur_y += 30 # Space for header
            
            # D-Pad Layout?? Or just 2x2? 
            # 2x2 Grid is efficient
            # [ < ] [ > ]
            # [ ^ ] [ v ]
            
            # Row 1: Pan (Jog @ 6000 steps/sec - Reliable High Speed)
            buttons.append(Button(bx_center - btn_sz - spacing//2, cur_y, btn_sz, btn_sz, "<", "J3:-6000"))
            buttons.append(Button(bx_center + spacing//2, cur_y, btn_sz, btn_sz, ">", "J3:6000"))
            
            # Row 2: Tilt (Jog @ 4000 steps/sec - Reliable High Speed)
            cur_y += btn_sz + spacing
            buttons.append(Button(bx_center - btn_sz - spacing//2, cur_y, btn_sz, btn_sz, "Up", "J4:4000"))
            buttons.append(Button(bx_center + spacing//2, cur_y, btn_sz, btn_sz, "Dn", "J4:-4000"))
            
            cur_y += btn_sz + 30 # Section Break
            
            # --- SHOOTER GROUP ---
            # Header drawn in loop
            cur_y += 30
            
            # Row 1: Pan (Jog @ 6000 steps/sec)
            # Tracker M1 now on Bottom
            buttons.append(Button(bx_center - btn_sz - spacing//2, cur_y, btn_sz, btn_sz, "<", "J1:-6000"))
            buttons.append(Button(bx_center + spacing//2, cur_y, btn_sz, btn_sz, ">", "J1:6000"))
            
            # Row 2: Tilt (Jog @ 4000 steps/sec)
            cur_y += btn_sz + spacing
            buttons.append(Button(bx_center - btn_sz - spacing//2, cur_y, btn_sz, btn_sz, "Up", "J2:4000"))
            buttons.append(Button(bx_center + spacing//2, cur_y, btn_sz, btn_sz, "Dn", "J2:-4000"))
            
            cur_y += btn_sz + 30 # Section Break

            # --- AUTO TRACKING ---
            cur_y += 30
            # Auto Track Button (Toggle) - Centered
            buttons.append(Button(bx_center - 25, cur_y, 50, 50, "AUTO", "AUTO_TRACK", is_toggle=True))
            
            cur_y += 50 + spacing
            # Pan Lock Button (Toggle)
            buttons.append(Button(bx_center - 25, cur_y, 50, 50, "LOCK", "PAN_LOCK", is_toggle=True))
            
            cur_y += 50 + spacing
            # AI Tracking Button (Toggle)
            buttons.append(Button(bx_center - 25, cur_y, 50, 50, "AI", "AI_TRACK", is_toggle=True, is_orange=True))
            
            cur_y += 50 + spacing
            # NUC Button (Manual Calibration)
            buttons.append(Button(bx_center - 25, cur_y, 50, 50, "NUC", "TRIGGER_NUC", is_toggle=False))
            
            cur_y += 50 + spacing
            # AI Mode Switch Button (Momentary)
            buttons.append(Button(bx_center - 25, cur_y, 50, 50, "MODE", "AI_MODE", is_toggle=False))

            # State for tracking throttling
            last_track_time = 0
            TRACK_INTERVAL = 0.016 # 60Hz Smooth Tracking
            
            # --- CORRECTED PD CONTROLLER STATE ---
            # Previous error for TRUE derivative calculation
            prev_err_x, prev_err_y = 0.0, 0.0
            d_filt_x, d_filt_y = 0.0, 0.0 # Filtered derivatives
            last_loop_time = time.time()
            
            # --- TARGET SMOOTHING (Anti-Jump) ---
            smooth_tgt_x, smooth_tgt_y = 128.0, 96.0 # Start at center
            TARGET_SMOOTH = 0.15 # 0.0=frozen, 1.0=instant (0.15 = heavy smoothing)
            
            # Conservative gains: KD >> KP for heavy damping on NEMA 23
            KP_PD = 25.0
            KD_PD = 120.0
            D_FILTER = 0.3 # 0.0 to 1.0 (Low = more smoothing)
            
            # Manual Hold State
            held_button = None
            last_hold_time = 0
            HOLD_INTERVAL = 0.05 # 20Hz updates for smooth manual hold
            
            # Oscillation Fix (Microstepping 6400):
            # Ratio 16x finer than original. 
            # 1 step = 0.468 deg ~= 2.7 pixels.
            # DEADBAND > Step/2 (~1.4px). Safe = 8px (~1.4 deg).
            # DEADBAND - Increased for stability in Auto-Track
            DEADBAND = 15

            # Resolve buttons for tracking loop
            btn_auto = next((b for b in buttons if b.text == "AUTO"), None)
            btn_lock = next((b for b in buttons if b.text == "LOCK"), None)
            btn_gear = next((b for b in buttons if b.cmd == "SETTINGS"), None)
            btn_ai = next((b for b in buttons if b.text == "AI"), None)
            btn_mode = next((b for b in buttons if b.text == "MODE"), None)
            
            # HW Command helper
            hw_cmd = None
            try:
                hw_cmd = t2pro_cmd.T2ProCmd()
                print("[MAIN] Hardware Command Link Established.")
            except Exception as e:
                print(f"[MAIN] Hardware Command Link Failed: {e}")

            # Device Info for Settings Page
            hw_serial = "Unknown"
            hw_fw_ver = "Unknown"
            info_requested = False

            # Mouse Callback
            def mouse_event(event, x, y, flags, param):
                nonlocal held_button, last_hold_time
                if event == cv2.EVENT_MOUSEMOVE:
                    for b in buttons:
                        b.hover = b.is_inside(x, y)
                elif event == cv2.EVENT_LBUTTONDOWN:
                    for b in buttons:
                        if b.is_inside(x, y):
                            if b.disabled:
                                continue # Block interaction
                            if b.is_toggle:
                                b.active = not b.active # Toggle state
                                if b.cmd == "AI_TRACK":
                                    thermal_ai.set_enabled(b.active)
                            else:
                                if b.cmd == "AI_MODE":
                                    thermal_ai.set_mode("PERSON" if thermal_ai.mode == "DRONE" else "DRONE")
                                    print(f"[AI] Switched to {thermal_ai.mode} mode.")
                                    b.clicked = True # Momentary visual feedback
                                elif b.cmd == "TRIGGER_NUC":
                                     b.clicked = True
                                     if use_t2pro_cmd:
                                         try:
                                             print("[CMD] Triggering NUC...")
                                             t2pro_cmd.shutter_calibration()
                                         except Exception as e:
                                             print(f"[ERR] NUC Failed: {e}")
                                else:
                                    # Check Lock for Manual Tilt (Support J or M commands)
                                    cmd_str = b.cmd.upper()
                                    is_tilt = "M2" in cmd_str or "M4" in cmd_str or "J2" in cmd_str or "J4" in cmd_str
                                    is_locked = btn_lock.active if btn_lock else False
                                    
                                    if is_tilt and is_locked:
                                        print("[LOCK] Vertical movement is locked. Unlock to move.")
                                    else:
                                        if is_tilt:
                                            print(f"[DEBUG] Sending Tilt Command: {b.cmd}")
                                        b.clicked = True # Momentary
                                        held_button = b
                                        serial_mgr.send_command(b.cmd) # Immediate Fire
                                        last_hold_time = time.time()
                elif event == cv2.EVENT_LBUTTONUP:
                    print(f"[DEBUG] Mouse Release. Stopping all manual jogs.")
                    # Emergency Stop for all 4 motors just in case
                    for i in [1, 2, 3, 4]:
                        serial_mgr.send_command(f"J{i}:0")
                        
                    held_button = None
                    for b in buttons:
                        if not b.is_toggle:
                            b.clicked = False
            
            cv2.namedWindow("T2Pro Enhanced", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("T2Pro Enhanced", TOTAL_WIDTH, VIDEO_HEIGHT)
            cv2.setMouseCallback("T2Pro Enhanced", mouse_event)
            
            
            frame_count = 0
            last_time = time.time()
            last_frame_received_time = time.time()
            actual_fps = 0.0
            
            try:
                while True:
                    # --- SAFETY WATCHDOG ---
                    # If no button is held in UI, ensure we aren't jogging (Firmware Safety)
                    # We can pulse J:0 occasionally if needed, but for now just rely on clean LBUTTONUP.
                    
                    # Atomic flush of latest_frame (simulated)
                    curr_frame = latest_frame
                    
                    if curr_frame:
                        latest_frame = None # Consume it so we don't re-process
                        last_frame_received_time = time.time()
                        
                        data, w, h = curr_frame
                        
                        try:
                            arr = np.frombuffer(data, dtype=np.uint16)
                        except ValueError:
                            continue

                        expected = w * h
                        if len(arr) == expected:
                            full_frame = arr.reshape((h, w))
                            
                            if h >= 192:
                                thermal_data = full_frame[0:192, :]
                                
                                # --- DIGITAL ZOOM (ROI CROP) ---
                                if zoom_factor > 1.0:
                                    h_raw, w_raw = thermal_data.shape
                                    center_x, center_y = w_raw // 2, h_raw // 2
                                    crop_w = int(w_raw / zoom_factor)
                                    crop_h = int(h_raw / zoom_factor)
                                    
                                    x1 = max(0, center_x - crop_w // 2)
                                    y1 = max(0, center_y - crop_h // 2)
                                    x2 = min(w_raw, x1 + crop_w)
                                    y2 = min(h_raw, y1 + crop_h)
                                    
                                    # Crop and Resize back to 256x192 for processing
                                    cropped = thermal_data[y1:y2, x1:x2]
                                    thermal_data = cv2.resize(cropped, (w_raw, h_raw), interpolation=cv2.INTER_CUBIC)

                                # --- AI PROCESSING (if enabled) ---
                                ai_detections = []
                                ai_target = None
                                hot_spot_blob = None
                                if btn_ai and btn_ai.active:
                                    _, ai_detections = thermal_ai.process_frame(thermal_data)
                                    if ai_detections:
                                        # Use primary detection as target
                                        det = ai_detections[0]
                                        box = det['box']
                                        cx_ai = int((box[0] + box[2]) / 2)
                                        cy_ai = int((box[1] + box[3]) / 2)
                                        ai_target = (cx_ai, cy_ai)

                                # Analyze stats (After Zoom = Active Contrast!)
                                thermal_data = np.asarray(thermal_data)
                                d_min = np.min(thermal_data)
                                d_max = np.max(thermal_data)
                                
                                # Use numpy if cv2 fails or is picky about format
                                max_idx = np.argmax(thermal_data)
                                min_idx = np.argmin(thermal_data)
                                h_t, w_t = thermal_data.shape
                                maxLoc = (int(max_idx % w_t), int(max_idx // w_t))
                                minLoc = (int(min_idx % w_t), int(min_idx // w_t))
                                minVal, maxVal = d_min, d_max
                                
                                # Spot Temperature (Center)
                                cy, cx = 192//2, 256//2
                                # Careful with bounds after zoom/resize
                                center_region = thermal_data[cy-2:cy+3, cx-2:cx+3]
                                spot_raw = np.mean(center_region)
                                
                                # FPS Calculation
                                frame_count += 1
                                current_time = time.time()
                                elapsed = current_time - last_time
                                if elapsed > 1.0:
                                    actual_fps = frame_count / elapsed
                                    frame_count = 0
                                    last_time = current_time
                                
                                if d_max > d_min:
                                    # Normalize - Simple and Robust
                                    norm = ((thermal_data.astype(np.float32) - d_min) / (d_max - d_min) * 255.0)
                                    norm = np.clip(norm, 0, 255).astype(np.uint8)
                                    norm = np.ascontiguousarray(norm)
                                    
                                    # --- STABLE HOT SPOT BLOB DETECTION ---
                                    # Remove vertical gradient by subtracting row-wise mean
                                    norm_float = thermal_data.astype(np.float32)
                                    row_means = norm_float.mean(axis=1, keepdims=True)
                                    detrended = norm_float - row_means  # Each row now has mean ~0
                                    
                                    # Normalize detrended data for thresholding
                                    dt_min, dt_max = detrended.min(), detrended.max()
                                    if dt_max > dt_min:
                                        detrended_norm = ((detrended - dt_min) / (dt_max - dt_min) * 255.0)
                                        detrended_norm = np.clip(detrended_norm, 0, 255).astype(np.uint8)
                                    else:
                                        detrended_norm = np.zeros_like(norm)
                                    
                                    # Threshold to find the hottest regions (relative to their row)
                                    _, thresh = cv2.threshold(detrended_norm, 200, 255, cv2.THRESH_BINARY)
                                    
                                    # MASKING: Ignore edges (Sky/Floor thermal artifacts)
                                    thresh[0:30, :] = 0      # Top 30 pixels
                                    thresh[-30:, :] = 0      # Bottom 30 pixels
                                    
                                    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                                    
                                    if contours:
                                        # Pick largest blob
                                        best_cnt = max(contours, key=cv2.contourArea)
                                        if cv2.contourArea(best_cnt) > 2:
                                            bx, by, bw, bh = cv2.boundingRect(best_cnt)
                                            M = cv2.moments(best_cnt)
                                            if M["m00"] != 0:
                                                bcx = int(M["m10"] / M["m00"])
                                                bcy = int(M["m01"] / M["m00"])
                                                hot_spot_blob = (bx, by, bw, bh, bcx, bcy)
                                    
                                    # --- FOCUS ASSIST CALCULATION (Raw Optic Check) ---
                                    focus_score = 0
                                    if use_focus:
                                        focus_score = cv2.Laplacian(norm, cv2.CV_64F).var()

                                    # --- MATRIX III (Temporal Smoothing) ---
                                    if use_matrix:
                                        if matrix_acc is None:
                                            matrix_acc = norm.astype(np.float32)
                                        else:
                                            cv2.accumulateWeighted(norm, matrix_acc, 0.5) # alpha=0.5
                                            norm = cv2.convertScaleAbs(matrix_acc)
                                    else:
                                        matrix_acc = None # Reset if disabled

                                    if use_blur:
                                        norm = cv2.medianBlur(norm, 3) 
                                        
                                    if use_clahe:
                                        norm = clahe.apply(norm)
                                        
                                    if use_sharpen:
                                        norm = cv2.filter2D(norm, -1, sharpen_kernel)
                                    
                                    if colormap_idx < 4:
                                        color_img = cv2.applyColorMap(norm, colormaps[colormap_idx])
                                    else:
                                        inv = cv2.bitwise_not(norm)
                                        color_img = cv2.cvtColor(inv, cv2.COLOR_GRAY2BGR)

                                    # Upscale
                                    upscale = cv2.resize(color_img, (1024, 768), interpolation=cv2.INTER_CUBIC)
                                    
                                    # --- AI DETECTION VISUALS ---
                                    if btn_ai and btn_ai.active and ai_detections:
                                        for det in ai_detections:
                                            box = det['box']
                                            # Scale boxes from 256x192 to 1024x768 (x4)
                                            x1, y1, x2, y2 = [int(v * 4) for v in box]
                                            cv2.rectangle(upscale, (x1, y1), (x2, y2), (0, 0, 255), 2)
                                            label = f"{thermal_ai.mode} ID:{det['id']} {det['conf']:.2f}"
                                            cv2.putText(upscale, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                                    
                                    # --- HOT SPOT VISUALS (Always Visible) ---
                                    if hot_spot_blob:
                                        bx, by, bw, bh, _, _ = hot_spot_blob
                                        # Scale from 256x192 to 1024x768 (x4)
                                        cv2.rectangle(upscale, (bx*4, by*4), ((bx+bw)*4, (by+bh)*4), (0, 255, 0), 2)
                                        # Label
                                        cv2.putText(upscale, "HOT SPOT", (bx*4, by*4-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                                    # Overlay: Status Indicators
                                    status_str = f"Zoom: {zoom_factor:.1f}x "
                                    if use_matrix: status_str += "MATRIX "
                                    if use_tracking: status_str += "TRACK "
                                    if use_clahe: status_str += "CLAHE "
                                    if use_sharpen: status_str += "SHARP "
                                    
                                    cv2.putText(upscale, status_str, (10, 750), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                                    
                                    # Focus Assist Overlay
                                    if use_focus:
                                        cv2.putText(upscale, f"Focus: {int(focus_score)}", (10, 720), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                                    # Center Crosshair
                                    cv2.line(upscale, (512, 384-20), (512, 384+20), (200, 200, 200), 1)

                                    if btn_auto.active:
                                        # Draw visual indicator for AUTO
                                        mode_text = f"AUTO TRACKING ({'AI' if btn_ai.active else 'HOT SPOT'})"
                                        cv2.putText(upscale, mode_text, (VIDEO_WIDTH//2 - 150, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 69, 255), 2)
                                        
                                        # Determine target: AI detection or Hot Spot Blob Centroid
                                        raw_tgt_x, raw_tgt_y = 128, 96 # Default center
                                        if btn_ai.active and ai_target:
                                            raw_tgt_x, raw_tgt_y = ai_target
                                        elif hot_spot_blob:
                                            raw_tgt_x, raw_tgt_y = hot_spot_blob[4], hot_spot_blob[5]
                                        else:
                                            raw_tgt_x, raw_tgt_y = maxLoc[0], maxLoc[1]
                                        
                                        # --- TARGET SMOOTHING (EMA to reject sudden jumps) ---
                                        smooth_tgt_x = smooth_tgt_x * (1 - TARGET_SMOOTH) + raw_tgt_x * TARGET_SMOOTH
                                        smooth_tgt_y = smooth_tgt_y * (1 - TARGET_SMOOTH) + raw_tgt_y * TARGET_SMOOTH
                                        target_x, target_y = int(smooth_tgt_x), int(smooth_tgt_y)

                                        # Draw Crosshair on Target (Scaled x4)
                                        tx, ty = target_x * 4, target_y * 4
                                        cv2.line(upscale, (tx, ty-15), (tx, ty+15), (0, 0, 255), 2)
                                        cv2.line(upscale, (tx-15, ty), (tx+15, ty), (0, 0, 255), 2)
                                        
                                        curr_time = time.time()
                                        dt_loop = max(0.001, curr_time - last_loop_time)
                                        last_loop_time = curr_time
                                        
                                        # Calculate error (simple: target - center)
                                        err_x = target_x - 128
                                        err_y = target_y - 96

                                        if curr_time - last_track_time > TRACK_INTERVAL:
                                            # --- BANG-BANG + COAST CONTROL ---
                                            # Simple: Move at fixed speed when FAR, stop when CLOSE
                                            # This prevents overshoot from continuously varying speed
                                            
                                            COAST_ZONE = 25  # Stop moving when within 25px to coast in
                                            MOVE_SPEED = 1500  # Fixed slow speed for tracking
                                            
                                            pan_speed = 0
                                            if abs(err_x) > COAST_ZONE:
                                                # Far away: move at fixed speed toward target
                                                pan_speed = MOVE_SPEED if err_x > 0 else -MOVE_SPEED
                                                serial_mgr.send_command(f"J3:{pan_speed}")
                                            elif abs(err_x) > DEADBAND:
                                                # In coast zone but not centered: stop and let it coast
                                                serial_mgr.send_command("J3:0")
                                            else:
                                                # Centered: ensure stopped
                                                serial_mgr.send_command("J3:0")
                                                
                                            # TILT (Needs more power vs gravity)
                                            TILT_SPEED = 3000 
                                            tilt_speed = 0
                                            pan_only = btn_lock.active if btn_lock else False
                                            
                                            if not pan_only:
                                                if abs(err_y) > COAST_ZONE:
                                                    # Same pattern as Pan: Positive error → Positive speed
                                                    tilt_speed = TILT_SPEED if err_y > 0 else -TILT_SPEED
                                                    serial_mgr.send_command(f"J4:{tilt_speed}")
                                                elif abs(err_y) > DEADBAND:
                                                    serial_mgr.send_command("J4:0")
                                                else:
                                                    serial_mgr.send_command("J4:0")
                                            
                                            # Debug logging to verify TILT logic
                                            if not pan_only and abs(err_y) > DEADBAND:
                                                 if not hasattr(main, 'last_tilt_log'): main.last_tilt_log = 0
                                                 if curr_time - main.last_tilt_log > 0.5:
                                                     print(f"[TILT DEBUG] err_y={err_y:.1f} cmd=J4:{tilt_speed} Locked={pan_only}")
                                                     main.last_tilt_log = curr_time
                                            
                                            last_track_time = curr_time
                                    cv2.line(upscale, (512-20, 384), (512+20, 384), (200, 200, 200), 1)
                                    
                                    stats = f"Center: {int(spot_raw)}"
                                    cv2.putText(upscale, stats, (520, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

                                    range_txt = f"Range: {d_min}-{d_max} | FPS: {actual_fps:.1f}"
                                    cv2.putText(upscale, range_txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)
                                    
                                    cv2.imshow("T2Pro Enhanced", upscale)
                                    
                                    # --- SIDEBAR COMPOSITION ---
                                    # Create Sidebar Canvas
                                    sidebar = np.zeros((VIDEO_HEIGHT, SIDEBAR_WIDTH, 3), dtype=np.uint8)
                                    sidebar[:] = C_BG # Fill BG
                                    
                                    # Helper: Draw aligned text
                                    def draw_label(img, txt, y, scale=0.5, color=C_TEXT, thick=1, font_type=cv2.FONT_HERSHEY_SIMPLEX):
                                        tsize, _ = cv2.getTextSize(txt, font_type, scale, thick)
                                        x = (SIDEBAR_WIDTH - tsize[0]) // 2
                                        cv2.putText(img, txt, (x, y), font_type, scale, color, thick, cv2.LINE_AA)
                                        
                                    draw_label(sidebar, "CONTROL", 15, 0.6, C_TEXT, 1)
                                    
                                    # Connection Status
                                    status_color = (0, 200, 0) if serial_mgr.connected else (200, 0, 0) 
                                    
                                    cv2.circle(sidebar, (20, 15), 4, status_color, -1, cv2.LINE_AA)
                                    
                                    # Divider
                                    cv2.line(sidebar, (30, 25), (SIDEBAR_WIDTH-30, 25), (200, 200, 200), 1)

                                    # Tracker Label
                                    draw_label(sidebar, "TRACKER", 30, 0.45, (100, 100, 100), 1)
                                    
                                    # Shooter Label
                                    draw_label(sidebar, "SHOOTER", 160, 0.45, (100, 100, 100), 1)
                                    
                                    # Automation Label
                                    draw_label(sidebar, "AUTOMATION", 290, 0.45, (100, 100, 100), 1)

                                    # --- TEENSY STATUS WARNING ---
                                    if not serial_mgr.connected:
                                        # Draw red warning text
                                        draw_label(sidebar, "TEENSY", 520, 0.5, (0, 0, 255), 2)
                                        draw_label(sidebar, "DISCONNECTED", 545, 0.5, (0, 0, 255), 2)
                                        
                                        # Disable motor buttons
                                        for b in buttons:
                                            # Motor buttons have commands starting with M or J (except Settings Gear '*')
                                            if b.cmd.startswith(("J", "M")):
                                                b.disabled = True
                                    else:
                                        # Re-enable if connection returns
                                        for b in buttons:
                                            b.disabled = False

                                    # Divider
                                    cv2.line(sidebar, (30, 500), (SIDEBAR_WIDTH-30, 500), (200, 200, 200), 1)

                                    # Combine
                                    combined = np.hstack((upscale, sidebar))
                                    
                                    # Draw Buttons on Combined
                                    for b in buttons:
                                        b.draw(combined)

                                    # --- SETTINGS OVERLAY ---
                                    if btn_settings and btn_settings.active:
                                        # Request HW Info once when opened
                                        if not info_requested and hw_cmd:
                                            try:
                                                sn = hw_cmd.get_device_info(t2pro_cmd.DeviceInfoType.DEV_INFO_GET_SN)
                                                hw_serial = sn.hex().upper()
                                                fw = hw_cmd.get_device_info(t2pro_cmd.DeviceInfoType.DEV_INFO_FW_BUILD_VERSION_INFO)
                                                hw_fw_ver = fw.decode('utf-8', errors='ignore').strip()
                                                info_requested = True
                                            except: pass
                                        
                                        # Draw Translucent Underlay
                                        overlay = combined.copy()
                                        cv2.rectangle(overlay, (50, 50), (VIDEO_WIDTH-50, VIDEO_HEIGHT-50), (30, 30, 30), -1)
                                        cv2.addWeighted(overlay, 0.85, combined, 0.15, 0, combined)
                                        
                                        # Draw Content
                                        ox, oy = 80, 100
                                        def draw_overlay_text(txt, size=0.6, color=(255, 255, 255), spacing_y=30):
                                            nonlocal oy
                                            cv2.putText(combined, txt, (ox, oy), cv2.FONT_HERSHEY_SIMPLEX, size, color, 1, cv2.LINE_AA)
                                            oy += spacing_y

                                        draw_overlay_text("SYSTEM SETTINGS & SHORTCUTS", 0.8, C_BTN_ACTIVE, 50)
                                        
                                        draw_overlay_text("--- SHORTCUTS ---", 0.5, (150, 150, 150), 30)
                                        draw_overlay_text("[Q] Quit Application", 0.5)
                                        draw_overlay_text("[C] Cycle Colormaps", 0.5)
                                        draw_overlay_text("[B] Toggle Median Blur", 0.5)
                                        draw_overlay_text("[M] Toggle Matrix III Smoothing", 0.5)
                                        draw_overlay_text("[H] Toggle CLAHE Contrast", 0.5)
                                        draw_overlay_text("[S] Toggle Sharpness", 0.5)
                                        draw_overlay_text("[X] Toggle Hotspot Tracking Visuals", 0.5)
                                        draw_overlay_text("[F] Toggle Focus Assist", 0.5)
                                        draw_overlay_text("[R] Force Shutter Calibration (NUC)", 0.5)
                                        draw_overlay_text("[P] Cycle Hardware Pseudo-Color", 0.5)
                                        draw_overlay_text("[G] Request Device Info", 0.5)
                                        draw_overlay_text("[+/-] Digital Zoom (1.0x - 15.0x)", 0.5)
                                        
                                        oy += 20
                                        draw_overlay_text("--- SYSTEM STATUS ---", 0.5, (150, 150, 150), 30)
                                        draw_overlay_text(f"Camera Serial: {hw_serial}", 0.5)
                                        draw_overlay_text(f"Firmware Build: {hw_fw_ver}", 0.5)
                                        draw_overlay_text(f"Current Colormap: {colormap_names[colormap_idx]}", 0.5)
                                        draw_overlay_text(f"Zoom level: {zoom_factor:.1f}x", 0.5)
                                        draw_overlay_text(f"Serial Link: {'CONNECTED' if serial_mgr.connected else 'ERROR'}", 0.5, (0, 255, 0) if serial_mgr.connected else (0, 0, 255))
                                        
                                        # Close instructions
                                        cv2.putText(combined, "Click Gear to Close", (VIDEO_WIDTH//2 - 70, VIDEO_HEIGHT-80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1, cv2.LINE_AA)
                                    else:
                                        info_requested = False # Reset so we poll again next time opened

                                    cv2.imshow("T2Pro Enhanced", combined)
                                    
                                    # --- SNAPSHOT FOR VISUAL FEEDBACK ---
                                    if frame_count == 30: # Save frame 30
                                        cv2.imwrite("ui_snapshot.png", combined)
                                        print("[UI] Saved reference snapshot to ui_snapshot.png")



                                else:
                                    cv2.imshow("T2Pro Enhanced", np.zeros((768, 1024, 3), dtype=np.uint8))
                        else:
                            pass
                    else:
                        # Watchdog Check
                        if time.time() - last_frame_received_time > 3.0:
                             print(f"[WATCHDOG] Warning: No frames for {time.time() - last_frame_received_time:.1f}s. Stream may be frozen.")
                             
                             # Auto-Reconnect Logic
                             if time.time() - last_frame_received_time > 5.0:
                                 print("[WATCHDOG] Attempting Auto-Reconnect...")
                                 libuvc.uvc_stop_streaming(devh)
                                 time.sleep(1.0)
                                 res = libuvc.uvc_start_streaming(devh, ctypes.byref(ctrl), c_frame_cb, None, 0)
                                 if res < 0:
                                     print(f"[WATCHDOG] Reconnect failed: {res}")
                                 else:
                                     print("[WATCHDOG] Reconnect success!")
                                 
                                 # Reset timer to give it a chance
                                 last_frame_received_time = time.time()
                             
                             time.sleep(0.5) # Don't spam

                    key = cv2.waitKey(10) & 0xFF
                    
                    # Window Close Check (X button)
                    if cv2.getWindowProperty("T2Pro Enhanced", cv2.WND_PROP_VISIBLE) < 1:
                        print("[SHUTDOWN] Window closed by user.")
                        break
                        
                    if key == ord('q'): break
                    elif key == ord('c'): colormap_idx = (colormap_idx + 1) % 6
                    elif key == ord('b'): use_blur = not use_blur
                    elif key == ord('h'):
                        use_clahe = not use_clahe
                        print(f"CLAHE: {use_clahe}")
                    elif key == ord('s'):
                        use_sharpen = not use_sharpen
                        print(f"Sharpen: {use_sharpen}")
                    elif key == ord('x'):
                        use_tracking = not use_tracking
                        print(f"Tracking: {use_tracking}")
                    elif key == ord('m'):
                        use_matrix = not use_matrix
                        print(f"Matrix III: {use_matrix}")
                    elif key == ord('f'):
                        use_focus = not use_focus
                        print(f"Focus Assist: {use_focus}")
                    elif key == ord('='): zoom_factor = min(15.0, zoom_factor + 0.5)
                    elif key == ord('-'): zoom_factor = max(1.0, zoom_factor - 0.5)
                    elif key == ord('r'):
                        if hw_cmd:
                             print("[HW] Forcing Shutter Calibration...")
                             try:
                                 hw_cmd.shutter_calibration()
                             except Exception as e:
                                 print(f"     Error: {e}")
                    elif key == ord('g'):
                        if hw_cmd:
                            print("[HW] Refreshing Device Info...")
                            try:
                                sn = hw_cmd.get_device_info(t2pro_cmd.DeviceInfoType.DEV_INFO_GET_SN)
                                print(f"     Serial: {sn.hex()}")
                                fw = hw_cmd.get_device_info(t2pro_cmd.DeviceInfoType.DEV_INFO_FW_BUILD_VERSION_INFO)
                                print(f"     FW Ver: {fw.decode('utf-8', errors='ignore')}")
                            except Exception as e:
                                print(f"     Error: {e}")
                    elif key == ord('p'):
                         if hw_cmd:
                             hw_color_idx = (hw_color_idx + 1) % len(hw_color_modes)
                             mode = hw_color_modes[hw_color_idx]
                             print(f"[HW] Setting Pseudo-Color: {mode.name}")
                             try:
                                 # 0 = preview path?
                                 hw_cmd.pseudo_color_set(0, mode)
                             except Exception as e:
                                 print(f"     Error: {e}")
            finally:
                serial_mgr.stop()
                if hw_cmd: hw_cmd.close()
                libuvc.uvc_stop_streaming(devh)
                cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
