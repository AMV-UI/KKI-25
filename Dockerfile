FROM ros:jazzy-ros-base

# Update apt and install pip
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-jazzy-joy ffmpeg python3-pip python3-venv python3-opencv && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir pyserial pymavlink py-trees inputs grpcio grpcio-tools setuptools==79.0.1 --break-system-packages && \
    mkdir -p /stubs/ && \
    cp -r /usr/local/lib/python3.12/dist-packages/py_trees /stubs/
