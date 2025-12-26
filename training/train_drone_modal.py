import modal
import os
import shutil
import random
import cv2
import numpy as np
from pathlib import Path

app = modal.App("thermal-drone-trainer")

# Define image with dependencies
image = (
    modal.Image.debian_slim()
    .apt_install("git")
    .run_commands("apt-get update && apt-get install -y libgl1-mesa-glx libglib2.0-0")
    .pip_install("ultralytics", "opencv-python-headless", "numpy", "tqdm")
)

# Persistent volume for results
vol = modal.Volume.from_name("drone-weights-vol", create_if_missing=True)

@app.function(image=image, volumes={"/data": vol}, gpu="T4", timeout=3600)
def train_drone_model():
    print("[1/4] Generating Synthetic Thermal Dataset...")
    
    # Setup directories
    base_dir = Path("/tmp/datasets/drone_synth")
    if base_dir.exists(): shutil.rmtree(base_dir)
    
    (base_dir / "images/train").mkdir(parents=True)
    (base_dir / "images/val").mkdir(parents=True)
    (base_dir / "labels/train").mkdir(parents=True)
    (base_dir / "labels/val").mkdir(parents=True)

    def create_synthetic_image(img_id, split):
        # 1. Background: Noise + Gradient (Simulate thermal environment)
        h, w = 640, 640
        img = np.zeros((h, w), dtype=np.uint8)
        
        # Gradient
        v_grad = np.linspace(20, 60, h)[:, None]
        img = img + v_grad + np.random.randint(-10, 10, (h, w))
        img = np.clip(img, 0, 255).astype(np.uint8)

        # 2. Add Distractors (Hotspots) - Class None (Background)
        # Large Gaussian blobs that look like sun/fire/body but NOT drones
        num_hotspots = random.randint(0, 3)
        for _ in range(num_hotspots):
            cx, cy = random.randint(50, 590), random.randint(50, 590)
            radius = random.randint(20, 60)
            cv2.circle(img, (cx, cy), radius, (200, 200, 200), -1)
            img = cv2.GaussianBlur(img, (55, 55), 0)

        # 3. Add Drone (Class 0)
        # Shape: Cross or H-shape, small, distinct edges (unlike soft blobs)
        labels = []
        if random.random() > 0.2: # 80% chance of drone
            cx, cy = random.randint(50, 590), random.randint(50, 590)
            size = random.randint(15, 30)
            
            # Draw Drone Shape (Active Heat Source - Bright White)
            color = 255
            # Body
            cv2.rectangle(img, (cx-size//2, cy-size//6), (cx+size//2, cy+size//6), color, -1)
            cv2.rectangle(img, (cx-size//6, cy-size//2), (cx+size//6, cy+size//2), color, -1)
            
            # Propeller Motors (4 points)
            pts = [
                (cx-size//2, cy-size//2), (cx+size//2, cy-size//2),
                (cx-size//2, cy+size//2), (cx+size//2, cy+size//2)
            ]
            for px, py in pts:
                cv2.circle(img, (px, py), size//4, 240, -1)
            
            # YOLO Format: class x_center y_center width height (normalized)
            # Bounding Box with some padding
            w_box = (size * 1.5) / w
            h_box = (size * 1.5) / h
            x_center = cx / w
            y_center = cy / h
            labels.append(f"0 {x_center:.6f} {y_center:.6f} {w_box:.6f} {h_box:.6f}")

        # Save
        cv2.imwrite(str(base_dir / f"images/{split}/{img_id}.jpg"), img)
        with open(base_dir / f"labels/{split}/{img_id}.txt", "w") as f:
            f.write("\n".join(labels))

    # Generate 1000 train, 200 val
    print("Generating 1000 Training Images...")
    for i in range(1000): create_synthetic_image(f"train_{i}", "train")
    
    print("Generating 200 Validation Images...")
    for i in range(200): create_synthetic_image(f"val_{i}", "val")

    # Create Data YAML
    yaml_content = f"""
    path: {base_dir}
    train: images/train
    val: images/val
    names:
      0: drone
    """
    with open(base_dir / "data.yaml", "w") as f:
        f.write(yaml_content)

    print("[2/4] Starting YOLOv8n Training...")
    from ultralytics import YOLO
    
    # Load pretrained model
    model = YOLO("yolov8n.pt") 
    
    # Train
    model.train(
        data=str(base_dir / "data.yaml"),
        epochs=15,
        imgsz=640,
        batch=16,
        project="/tmp/runs",
        name="drone_model",
        exist_ok=True,
        verbose=True
    )

    print("[3/4] Saving Best Weights...")
    best_weights = Path("/tmp/runs/drone_model/weights/best.pt")
    target_path = Path("/data/best_drone.pt")
    
    if best_weights.exists():
        shutil.copy(best_weights, target_path)
        print(f"SUCCESS: Weights saved to Volume at {target_path}")
        return True
    else:
        print("ERROR: Training failed, best.pt not found.")
        return False

@app.local_entrypoint()
def main():
    print("Launching Remote GPU Training on Modal...")
    success = train_drone_model.remote()
    
    if success:
        print("Training Complete! Downloading weights...")
        # Download from volume
        with open("training/best_drone.pt", "wb") as f:
            for chunk in vol.read_file("best_drone.pt"):
                f.write(chunk)
        print("Done! Weights saved to local: training/best_drone.pt")
    else:
        print("Training Failed.")
