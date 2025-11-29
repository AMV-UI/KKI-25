#!/usr/bin/env bash

tmux new-session -d "source install/local_setup.bash; ros2 run core_control movement_controller"
tmux split-window -v "source install/local_setup.bash; ros2 run core_control movement_simulator"
tmux split-window -h "source install/local_setup.bash; ros2 run core_control channel_simulator"
tmux -2 attach-session -d
