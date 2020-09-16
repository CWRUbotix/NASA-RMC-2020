#!/bin/bash

set -e # Exit if any command fails

PACKAGES=(
    # Kalman filters for robot localization
    ros-$ROS_DISTRO-robot-localization

    # rqt provides useful visualizer tools
    ros-$ROS_DISTRO-rqt
    ros-$ROS_DISTRO-rqt-common-plugins
    ros-$ROS_DISTRO-rqt-robot-plugins

    # Plotjuggler is a very cool plotting software
    ros-$ROS_DISTRO-plotjuggler

    # Used to drive simulated robot
    ros-$ROS_DISTRO-ros-controllers

    # Gazebo is the simulation used with ros
    # These packages depend on Gazebo and will install it automatically
    ros-$ROS_DISTRO-gazebo-ros-pkgs
    ros-$ROS_DISTRO-gazebo-ros-control

    # Needed to calculate point clouds of obstacles
    ros-$ROS_DISTRO-ros-numpy
)

sudo apt update # Make sure package list is up to date
sudo apt install ${PACKAGES[@]} # Install all ros packages
