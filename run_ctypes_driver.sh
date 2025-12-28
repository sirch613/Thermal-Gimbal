#!/bin/bash
LOCKFILE="/tmp/t2pro_launcher.lock"

# --- Prevent Duplicate Launchers ---
if [ -f "$LOCKFILE" ]; then
    OLDPID=$(cat "$LOCKFILE")
    if ps -p "$OLDPID" > /dev/null 2>&1; then
        echo "[ERROR] Another launcher is already running (PID $OLDPID). Exiting."
        exit 1
    else
        echo "[WARN] Stale lock file found. Cleaning up."
        rm -f "$LOCKFILE"
    fi
fi

# Write our PID to lockfile
echo $$ > "$LOCKFILE"
trap "rm -f $LOCKFILE" EXIT

echo "Running T2Pro Ctypes Driver (Requires Sudo)..."
while true; do
    echo "[LAUNCHER] Starting Driver..."
    python3 -u t2pro_ctypes_driver.py 2>&1 | tee -a launch_log.txt
    EXIT_CODE=$?
    echo "[LAUNCHER] Driver exited with code $EXIT_CODE. Restarting in 2 seconds..."
    sleep 2
done
