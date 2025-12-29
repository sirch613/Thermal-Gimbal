import os
import json
import cv2
import numpy as np
from tqdm import tqdm
from pathlib import Path

def convert_anti_uav(data_root, output_dir):
    """
    Converts Anti-UAV dataset (videos + JSON) to YOLO frames + labels.
    Expected structure:
    data_root/
        sequence_1/
            IR.mp4 (or video.mp4)
            target_info.json
    """
    data_root = Path(data_root)
    output_dir = Path(output_dir)
    img_out = output_dir / "images"
    lbl_out = output_dir / "labels"
    img_out.mkdir(parents=True, exist_ok=True)
    lbl_out.mkdir(parents=True, exist_ok=True)

    sequences = [d for d in data_root.iterdir() if d.is_dir()]
    print(f"Found {len(sequences)} sequences.")

    for seq in tqdm(sequences):
        json_path = seq / "target_info.json"
        if not json_path.exists(): continue

        with open(json_path, 'r') as f:
            data = json.load(f)

        # Anti-UAV format: data['gt_rect'] is [[x, y, w, h], ...]
        # data['exist'] is [0, 1, ...] visibility flag
        gt_rects = data.get('gt_rect', [])
        exist_flags = data.get('exist', [1] * len(gt_rects))

        # Find the video file (could be IR.mp4 or video.mp4)
        video_path = seq / "IR.mp4"
        if not video_path.exists():
            video_path = seq / "video.mp4"
        if not video_path.exists(): continue

        cap = cv2.VideoCapture(str(video_path))
        frame_idx = 0
        
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret or frame_idx >= len(gt_rects):
                break

            if exist_flags[frame_idx] == 1:
                # Valid target present
                x, y, w, h = gt_rects[frame_idx]
                
                # YOLO Normalization
                img_h, img_w = frame.shape[:2]
                xc = (x + w/2) / img_w
                yc = (y + h/2) / img_h
                wn = w / img_w
                hn = h / img_h

                # Save Frame (Every 5th frame to avoid redundancy)
                if frame_idx % 5 == 0:
                    name = f"{seq.name}_{frame_idx:04d}"
                    cv2.imwrite(str(img_out / f"{name}.jpg"), frame)
                    with open(lbl_out / f"{name}.txt", "w") as f_lbl:
                        f_lbl.write(f"0 {xc:.6f} {yc:.6f} {wn:.6f} {hn:.6f}")

            frame_idx += 1
        cap.release()

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 3:
        print("Usage: python prepare_anti_uav.py <data_root> <output_dir>")
    else:
        convert_anti_uav(sys.argv[1], sys.argv[2])
