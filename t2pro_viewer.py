import cv2
import sys
import time

def list_ports():
    """
    Test the ports and returns a tuple with the available ports and the ones that are working.
    """
    non_working_ports = []
    dev_port = 0
    working_ports = []
    available_ports = []
    while len(non_working_ports) < 6: # if there are more than 5 non working ports stop the testing. 
        camera = cv2.VideoCapture(dev_port)
        if not camera.isOpened():
            non_working_ports.append(dev_port)
            print("Port %s is not working." %dev_port)
        else:
            is_reading, img = camera.read()
            w = camera.get(3)
            h = camera.get(4)
            if is_reading:
                print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                working_ports.append(dev_port)
            else:
                print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                available_ports.append(dev_port)
        dev_port +=1
    return working_ports, available_ports, non_working_ports

def main():
    print("--- T2 Pro Viewer / Camera Diagnostics ---")
    print("Commands:")
    print("  'q': Quit")
    print("  'n': Next Camera")
    print("  'r': Force Resolution (Cycle 256x192 / 640x480)")
    print("------------------------------------------")

    current_index = 0
    cap = None
    
    # Try to find a working initial camera starting from 0
    for i in range(5):
        temp = cv2.VideoCapture(i)
        if temp.isOpened():
            current_index = i
            temp.release()
            print(f"Found initial working camera at index {current_index}")
            break
        else:
             print(f"Index {i} failed to open.")
    
    cap = cv2.VideoCapture(current_index)
    # T2Pro Specific: Try to force small resolution
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 192)

    if not cap.isOpened():
        print(f"Failed to open Initial Camera {current_index}.")
    
    cv2.namedWindow("T2 Pro Feed", cv2.WINDOW_NORMAL)
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            # If reading fails, just show a blank black image
            import numpy as np
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, f"Cam {current_index}: No Frame", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.imshow("T2 Pro Feed", frame)
            
            # Short sleep to prevent CPU burn on failure
            key = cv2.waitKey(100)
        else:
            # Info overlay
            h, w = frame.shape[:2]
            cv2.putText(frame, f"Index: {current_index} | Res: {w}x{h}", (10, 30), cv2.FONT_HERSHEY_PLAIN, 1.5, (0, 255, 0), 2)
            cv2.imshow("T2 Pro Feed", frame)
            key = cv2.waitKey(1)

        key = key & 0xFF
        if key == ord('q'):
            break
        elif key == ord('n'):
            # Switch to next camera
            print("Switching camera...")
            if cap:
                cap.release()
            current_index = (current_index + 1) % 5
            # Skip if we know it doesn't exist? No, just try it.
            cap = cv2.VideoCapture(current_index)
            # Try to force resolution for T2Pro if it's the target
            # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
            # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 192)
            print(f"Attempting Camera Index {current_index}")
        elif key == ord('r'):
             print("Attempting to force 256x192...")
             cap.set(cv2.CAP_PROP_FRAME_WIDTH, 256)
             cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 192)

    if cap:
        cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
