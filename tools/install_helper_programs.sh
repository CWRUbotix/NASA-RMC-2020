#!/bin/bash

set -e # Exit if any command fails

PACKAGES=(
    # Allows use of catkin build and other catkin tools
    python3-catkin-tools

    # Pip
    python3-pip

    # Tells you cpu temps (run `sensors`)
    lm-sensors

    # Cool cpu usage monitor
    htop

    # Terminal for automodule
    xterm

    # Enables ssh from a computer onto another device
    openssh-server

    # For wxPython

        python3-dev # (for your version of python)
        libgtk-3-dev
)

sudo apt update # Make sure package list is up to date
sudo apt install ${PACKAGES[@]} # Install all packages

# Update pip and install packages
# We use py3 most of the time but some
# ros packages only work with py2
python3 -m pip install --upgrade pip
python3 -m pip install -r requirements.txt
