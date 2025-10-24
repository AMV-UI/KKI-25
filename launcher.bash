#!/usr/bin/env bash

sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyUSB0

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

if [ -f "$WORKSPACE_ROOT/install/local_setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/local_setup.bash"
    echo "[INFO] Sourced: $WORKSPACE_ROOT/install/local_setup.bash"
else
    echo "[ERROR] local_setup.bash not found in $WORKSPACE_ROOT/install/"
    exit 1
fi

echo "[INFO] Launching core_launcher/launcher.py..."
exec ros2 launch core_launcher launcher.py
