import cv2
import numpy as np
from ultralytics import YOLO
import torch
import warnings
warnings.filterwarnings("ignore")


class ThermalAI:
    def __init__(self, model_path='training/best_drone_real.pt', confidence=0.4):
        # Hardware Acceleration Check
        self.device = 'mps' if torch.backends.mps.is_available() else 'cpu'
        print(f"[AI] Hardware Acceleration: {self.device.upper()}")

        print(f"[AI] Loading Custom Drone Model: {model_path}...")
        try:
            self.model = YOLO(model_path)
            self.model.to(self.device)
        except:
             print("[AI] Custom model not found, falling back to yolov8n.pt")
             self.model = YOLO('yolov8n.pt')
             self.model.to(self.device)
             
        self.confidence = confidence
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.mode = "DRONE"

    def set_mode(self, mode):
        self.mode = mode.upper()
        if self.mode == "DRONE":
            print("[AI] Switching to DRONE model (Real IR)...")
            try:
                self.model = YOLO('training/best_drone_real.pt')
                self.model.to(self.device)
            except:
                print("[AI] best_drone_real.pt not found!")
        elif self.mode == "PERSON":
            print("[AI] Switching to PERSON model...")
            self.model = YOLO('yolov8n.pt')
            self.model.to(self.device)
        
    def process_frame(self, raw_16bit):
        """
        Input: Raw 16-bit thermal data (e.g., 256x192)
        Output: (color_frame, list of detections)
        """
        # 1. Percentile-based Normalization (2/98)
        # Bypasses outliers (sun, hot spots) to maintain contrast on interest targets
        vmin = np.percentile(raw_16bit, 2)
        vmax = np.percentile(raw_16bit, 98)
        
        if vmax <= vmin:
            # Fallback for flat scenes
            vmin, vmax = np.min(raw_16bit), np.max(raw_16bit)
            if vmax <= vmin: vmax = vmin + 1
            
        thermal_norm = np.clip(raw_16bit, vmin, vmax)
        thermal_8bit = ((thermal_norm - vmin) / (vmax - vmin) * 255).astype(np.uint8)
        
        # 2. CLAHE Enhancement
        enhanced = self.clahe.apply(thermal_8bit)
        
        # 3. Create BGR for YOLO (expects 3-channel)
        color_frame = cv2.cvtColor(enhanced, cv2.COLOR_GRAY2BGR)
        
        # 4. YOLO + ByteTrack
        results = self.model.track(
            color_frame,
            persist=True,
            tracker="bytetrack.yaml",
            verbose=False,
            conf=self.confidence
        )
        
        detections = []
        if results and results[0].boxes is not None:
            boxes = results[0].boxes
            if boxes.id is not None:
                # Tracked objects
                xyxy = boxes.xyxy.cpu().numpy()
                ids = boxes.id.cpu().numpy().astype(int)
                confs = boxes.conf.cpu().numpy()
                cls = boxes.cls.cpu().numpy().astype(int)
                
                for i in range(len(ids)):
                    detections.append({
                        'box': xyxy[i],
                        'id': ids[i],
                        'conf': confs[i],
                        'class': cls[i]
                    })
                    
        return color_frame, detections

    def draw_detections(self, frame, detections, scale_factor=1.0):
        """
        Draws RED boxes and IDs on the frame.
        scale_factor: Used if drawing on an upscaled high-res window.
        """
        for det in detections:
            x1, y1, x2, y2 = det['box'] * scale_factor
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
            
            # Bold RED Box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            
            # Label
            label = f"ID:{det['id']} {det['conf']:.2f}"
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        return frame
