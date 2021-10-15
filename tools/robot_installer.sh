#!/bin/bash

# check if anything else is messing with dpkg don't run until updates have stopped
ps -ef | grep dpkg


sudo apt install curl
sudo apt install git

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-melodic-desktop


echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc



sudo apt install python-rosdep
sudo rosdep init
rosdep update


mkdir catkin_ws 

cd catkin_ws
mkdir src
cd src

repository="https://github.com/cwruRobotics/NASA-RMC-2020.git"
localFolder=""
branch="master"

git clone --depth 1 -b "$branch" "$repository"  

cd NASA-RMC-2020

git submodule update --init

rm -r glenn_simulation

cd tools

./install_helper_programs.sh

source ~/.bashrc

./install_ros_deps.sh

sudo apt-get upgrade

cd ../../../

catkin build            

cd ../   

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

echo "export ROS_LOG_DIR=~/../../media/cwrubotix/USB_BAGGING/log/" >> ~/.bashrc
source ~/.bashrc