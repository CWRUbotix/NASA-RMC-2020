#!/bin/bash

set -e # Exit if any command fails

PACKAGES=(
    # Allows use of catkin build and other catkin tools
    python-catkin-tools

    # Pip
    python-pip
    python3-pip

    # Tells you cpu temps (run `sensors`)
    lm-sensors

    # Cool cpu usage monitor
    htop

    # Terminal for automodule
    xterm

    # Enables ssh from a computer onto another device
    openssh-server
)

sudo apt update # Make sure package list is up to date
sudo apt install ${PACKAGES[@]} # Install all packages

# Update pip and install packages
# We use py3 most of the time but some
# ros packages only work with py2
python -m pip install --user --upgrade pip
python3 -m pip3 install --user --upgrade pip

python -m pip install -r requirements.txt
python3 -m pip3 install -r requirements.txt
