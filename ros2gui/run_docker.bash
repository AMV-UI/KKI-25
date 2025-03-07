# If not working, try sudo rm -rf /tmp/.docker.xauth
# add --runtime=nvidia to docker run options if need GPU

sudo xhost local:root # Unsafe but works

XAUTH=/tmp/.docker.xauth

sudo docker run -it \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    vane/ros2-humble:multi-arch \
    bash
