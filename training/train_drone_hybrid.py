import modal
import os
import shutil
import random
import cv2
import numpy as np
from pathlib import Path

app = modal.App("thermal-drone-trainer-hybrid")

# Define image with dependencies
image = (
    modal.Image.debian_slim()
    .apt_install("git")
    .run_commands("apt-get update && apt-get install -y libgl1-mesa-glx libglib2.0-0")
    .pip_install("ultralytics", "opencv-python-headless", "numpy", "tqdm")
)

# Persistent volume for datasets and results
vol = modal.Volume.from_name("drone-weights-vol", create_if_missing=True)

@app.function(image=image, volumes={"/data": vol}, gpu="T4:2", timeout=10800)
def train_drone_parallel():
    import concurrent.futures
    print("[1/3] Preparing Dataset (Multi-threaded staging)...")
    base_dir = Path("/tmp/datasets/drone_real")
    if base_dir.exists(): shutil.rmtree(base_dir)
    
    img_train = base_dir / "images/train"
    img_val = base_dir / "images/val"
    lbl_train = base_dir / "labels/train"
    lbl_val = base_dir / "labels/val"
    
    for d in [img_train, img_val, lbl_train, lbl_val]: d.mkdir(parents=True)

    real_uav_path = Path("/data/real_uav")
    images = list((real_uav_path / "images").glob("*.jpg"))
    random.shuffle(images)
    
    split_idx = int(len(images) * 0.8)
    train_imgs = images[:split_idx]
    val_imgs = images[split_idx:]
    
    def stage_file(img_path, split):
        dest_img = base_dir / f"images/{split}" / img_path.name
        shutil.copy(img_path, dest_img)
        lbl_path = real_uav_path / "labels" / f"{img_path.stem}.txt"
        if lbl_path.exists():
            shutil.copy(lbl_path, base_dir / f"labels/{split}" / f"{img_path.stem}.txt")
            return 1
        return 0

    print(f"Staging {len(images)} files on local SSD...")
    with concurrent.futures.ThreadPoolExecutor(max_workers=16) as executor:
        futures = []
        for img in train_imgs: futures.append(executor.submit(stage_file, img, "train"))
        for img in val_imgs: futures.append(executor.submit(stage_file, img, "val"))
        results = [f.result() for f in concurrent.futures.as_completed(futures)]
    
    print(f"Staging complete: {sum(results)} frames ready.")

    yaml_content = f"path: {base_dir}\ntrain: images/train\nval: images/val\nnames: {{0: 'drone'}}\n"
    with open(base_dir / "data.yaml", "w") as f:
        f.write(yaml_content)

    print("[2/3] Training (2x T4 DDP Enabled)...")
    from ultralytics import YOLO
    model = YOLO("yolov8n.pt") 
    # Multi-GPU via device='0,1'
    model.train(data=str(base_dir/"data.yaml"), epochs=50, imgsz=640, device='0,1', batch=128)

    print("[3/3] Uploading results...")
    best_weights = Path("runs/detect/train/weights/best.pt")
    if best_weights.exists():
        shutil.copy(best_weights, "/data/best_drone.pt")
        return True
    return False

@app.local_entrypoint()
def main():
    if train_drone_parallel.remote():
        print("Success! Downloading...")
        with open("training/best_drone.pt", "wb") as f:
            for chunk in vol.read_file("best_drone.pt"): f.write(chunk)
