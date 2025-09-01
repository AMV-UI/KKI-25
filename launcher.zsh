#!/usr/bin/env zsh

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

if [ -f "$WORKSPACE_ROOT/install/local_setup.zsh" ]; then
    source "$WORKSPACE_ROOT/install/local_setup.zsh"
    echo "[INFO] Sourced: $WORKSPACE_ROOT/install/local_setup.zsh"
else
    echo "[ERROR] local_setup.zsh not found in $WORKSPACE_ROOT/install/"
    exit 1
fi

echo "[INFO] Launching core_launcher/launcher.py..."
exec ros2 launch core_launcher launcher.py

