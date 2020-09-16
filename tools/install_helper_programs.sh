#!/bin/bash

set -e # Exit if any command fails

PACKAGES=(
    # Allows use of catkin build and other catkin tools
    python-catkin-tools

    # Pip
    python3-pip

    # Tells you cpu temps (run `sensors`)
    lm-sensors

    # Cool cpu usage monitor
    htop

    # Display for Gazebo
    xterm
)

sudo apt update # Make sure package list is up to date
sudo apt install ${PACKAGES[@]} # Install all packages
