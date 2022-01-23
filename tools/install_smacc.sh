#!/bin/bash

# For installing smacc and smacc-viewer

curl -s https://b0e12e65a4f16bfc4594206c69dce2a49a5eabd04efb7540:@packagecloud.io/install/repositories/reelrbtx/SMACC_viewer/script.deb.sh | sudo bash

#Seperate file because this takes like 50 years
sudo python3 -m pip install wxPython --verbose

curl -s https://packagecloud.io/install/repositories/reelrbtx/SMACC/script.deb.sh | sudo bash

set -e # Exit if any command fails

PACKAGES=(
# SMACC Viewer
ros-$ROS_DISTRO-smacc-viewer

# General SMACC dependicies
ros-$ROS_DISTRO-move-group-interface-client
ros-$ROS_DISTRO-backward-local-planner
ros-$ROS_DISTRO-undo-path-global-planner
ros-$ROS_DISTRO-backward-global-planner
ros-$ROS_DISTRO-forward-local-planner
ros-$ROS_DISTRO-forward-global-planner
ros-$ROS_DISTRO-move-base-z-client-plugin
ros-$ROS_DISTRO-ros-timer-client
ros-$ROS_DISTRO-ros-publisher-client
ros-$ROS_DISTRO-multirole-sensor-client
ros-$ROS_DISTRO-keyboard-client
ros-$ROS_DISTRO-sr-event-countdown
ros-$ROS_DISTRO-sr-conditional
ros-$ROS_DISTRO-sr-all-events-go
ros-$ROS_DISTRO-smacc-runtime-test
ros-$ROS_DISTRO-smacc
ros-$ROS_DISTRO-smacc-msgs
)

sudo apt upgrade # Make sure package list is up to date
sudo apt install ${PACKAGES[@]} # Install all ros packages