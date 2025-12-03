#!/usr/bin/env bash

tmux split-window -h "ros2 run core_control channel_simulator"

tmux -2 attach-session -d
