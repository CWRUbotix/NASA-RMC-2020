#!/bin/bash

# Code to setup the robot.
# This file should be run from the home of the robot
# It currently sets up the robot on ROS Melodic

# Check if anything else is messing with dpkg don't run until updates have stopped
ps -ef | grep dpkg

# Installs curl and git
sudo apt install curl
sudo apt install git

# ROS setup and installation
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop

# Adds this to the .bashrc and sources it
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc


# Installs python ROS depencies
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

# Creates catkin_ws folder and the src folder with catkinn_ws
mkdir catkin_ws 
cd catkin_ws
mkdir src
cd src

# Clones only the head of the main branch
repository="https://github.com/cwruRobotics/NASA-RMC-2020.git"
localFolder=""
branch="master"
git clone --depth 1 -b "$branch" "$repository"  

# Updates the submodule
cd NASA-RMC-2020
git submodule update --init

# Removes the simulation folder
rm -r glenn_simulation

# Installs helper programs and ROS depencies
cd tools
./install_helper_programs.sh
source ~/.bashrc
./install_ros_deps.sh

# Upgrades all packages
sudo apt-get upgrade

# Builds the code
cd ../../../
catkin build            
cd ../   

# Adds the lines to the .bashrc and sources the .bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
echo "export ROS_LOG_DIR=/media/cwrubotix/USB_BAGGING/log" >> ~/.bashrc
source ~/.bashrc