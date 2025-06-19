#!/bin/bash

# Build the workspace
colcon build --symlink-install

# Check if build succeeded
if [ $? -eq 0 ]; then
    echo -e "\n\033[32mBuild successful! Sourcing setup.bash...\033[0m"
    # Source the setup file
    source install/setup.bash
else
    echo -e "\n\033[31mBuild failed! Not sourcing setup.bash.\033[0m"
    exit 1
fi
