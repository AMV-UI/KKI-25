#!/usr/bin/env bash

tmux new-session -d "ros2 run core_control movement_controller"
tmux split-window -v "ros2 run core_control movement_simulator"
tmux split-window -h "ros2 run core_control channel_simulator"

tmux -2 attach-session -d
