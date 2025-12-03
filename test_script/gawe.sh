#!/bin/bash

# Build the workspace
colcon build --symlink-install

# Check if build succeeded
if [ $? -eq 0 ]; then
  echo -e "\n\033[32mBuild successful! Detecting shell and sourcing setup file...\033[0m"

  if [ -n "$ZSH_VERSION" ]; then
    source install/local_setup.zsh
    echo -e "\033[34mSourced setup.zsh for zsh\033[0m"
  elif [ -n "$BASH_VERSION" ]; then
    source install/local_setup.bash
    echo -e "\033[34mSourced setup.bash for bash\033[0m"
  else
    echo -e "\033[33mUnknown shell. Please source setup file manually.\033[0m"
  fi

else
  echo -e "\n\033[31mBuild failed! Not sourcing setup file.\033[0m"
  exit 1
=======
    echo -e "\n\033[32mBuild successful! Sourcing setup.bash...\033[0m"
    # Source the setup file
    source install/setup.bash
else
    echo -e "\n\033[31mBuild failed! Not sourcing setup.bash.\033[0m"
    exit 1
fi
