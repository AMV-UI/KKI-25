#!/bin/bash

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
STUBS_DIR="$HOME/.local/share/ros2_stubs"

mkdir -p "$STUBS_DIR"

echo "========================================="
echo " Starting ROS2 Docker Environment        "
echo " Mirroring : $WORKSPACE_DIR              "
echo " Purpose of mirroring : ROS2 symlinks (like windows shortcuts) breaks"
echo "========================================="

docker run -it --rm \
    --network=host \
    -v "$WORKSPACE_DIR:$WORKSPACE_DIR" \
    -v "$STUBS_DIR:/stubs" \
    -v "/dev:/dev" \
    -v "/run/udev:/run/udev:ro" \
    -v "/sys:/sys:ro" \
    --privileged \
    -w "$WORKSPACE_DIR" \
    ros-jazzy-rov \
    bash -c "if [ -f \"$WORKSPACE_DIR/install/setup.bash\" ]; then source \"$WORKSPACE_DIR/install/setup.bash\"; else source /opt/ros/jazzy/setup.bash; fi; exec bash"
