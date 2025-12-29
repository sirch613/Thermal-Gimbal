import modal
import os
import shutil
import json
import cv2
from pathlib import Path

app = modal.App("thermal-dataset-manager")

# Image with tools for downloading and unzipping
image = (
    modal.Image.debian_slim()
    .apt_install("git", "wget", "unzip", "curl", "libgl1-mesa-glx", "libglib2.0-0")
    .pip_install("tqdm", "requests", "gdown", "opencv-python-headless", "numpy")
)

vol = modal.Volume.from_name("drone-weights-vol", create_if_missing=True)

@app.function(image=image, volumes={"/data": vol}, timeout=3600)
def process_anti_uav():
    """
    Downloads and converts Anti-UAV300 dataset from Google Drive.
    """
    data_dir = Path("/data/real_uav")
    (data_dir / "images").mkdir(parents=True, exist_ok=True)
    (data_dir / "labels").mkdir(parents=True, exist_ok=True)

    backup_zip = Path("/data/anti_uav300.zip")
    
    if not backup_zip.exists():
        print("Downloading Anti-UAV300 from Google Drive...")
        import gdown
        file_id = "1NPYaop35ocVTYWHOYQQHn8YHsM9jmLGr"
        gdown.download(id=file_id, output=str(backup_zip), quiet=False)

    print("Extracting dataset...")
    tmp_extract = Path("/tmp/anti_uav")
    if tmp_extract.exists(): shutil.rmtree(tmp_extract)
    tmp_extract.mkdir(parents=True)
    
    os.system(f"unzip -q {backup_zip} -d {tmp_extract}")
    
    print("Processing videos and annotations...")
    # Anti-UAV300 structure: test/sequence_name/infrared.mp4 and infrared.json
    count = 0
    for root, dirs, files in os.walk(tmp_extract):
        if "infrared.mp4" in files and "infrared.json" in files:
            seq_dir = Path(root)
            json_path = seq_dir / "infrared.json"
            video_path = seq_dir / "infrared.mp4"
            
            with open(json_path, 'r') as f:
                data = json.load(f)
            
            gt_rects = data.get('gt_rect', [])
            exist_flags = data.get('exist', [1] * len(gt_rects))
            
            cap = cv2.VideoCapture(str(video_path))
            f_idx = 0
            while cap.isOpened():
                ret, frame = cap.read()
                if not ret or f_idx >= len(gt_rects): break
                
                # Ingest every 10th frame where drone exists
                if exist_flags[f_idx] == 1 and f_idx % 10 == 0:
                    x, y, w, h = gt_rects[f_idx]
                    img_h, img_w = frame.shape[:2]
                    
                    # YOLO Norm
                    xc, yc = (x + w/2) / img_w, (y + h/2) / img_h
                    wn, hn = w / img_w, h / img_h
                    
                    name = f"antiuav_{seq_dir.name}_{f_idx:04d}"
                    cv2.imwrite(str(data_dir / "images" / f"{name}.jpg"), frame)
                    with open(data_dir / "labels" / f"{name}.txt", "w") as fl:
                        fl.write(f"0 {xc:.6f} {yc:.6f} {wn:.6f} {hn:.6f}")
                    count += 1
                f_idx += 1
            cap.release()
            print(f"Processed {seq_dir.name}: {count} total frames so far.")

    print(f"SUCCESS: Ingested {count} real drone frames into /data/real_uav")
    vol.commit()
    return count

@app.local_entrypoint()
def main():
    print("Starting Anti-UAV300 Ingestion on Modal...")
    count = process_anti_uav.remote()
    print(f"Finished! {count} real drone frames are ready in the cloud.")
