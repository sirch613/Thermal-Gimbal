#!/bin/bash
echo "Running T2Pro Ctypes Driver (Requires Sudo)..."
while true; do
    echo "[LAUNCHER] Starting Driver..."
    python3 -u t2pro_ctypes_driver.py 2>&1 | tee -a launch_log.txt
    EXIT_CODE=$?
    echo "[LAUNCHER] Driver exited with code $EXIT_CODE. Restarting in 2 seconds..."
    sleep 2
done
