import t2pro_cmd
import time

print("Attempting to connect to T2Pro for NUC...")
try:
    with t2pro_cmd.T2ProCmd() as cmd:
        print("Connected! Sending Shutter Calibration...")
        cmd.shutter_calibration()
        print("Success! Command sent.")
except Exception as e:
    print(f"Failed: {e}")
