#!/bin/bash
cd $(dirname $0)
#鱼香ros配置rosdep
# wget http://fishros.com/install -O fishros && . fishros
sleep 0.5s
source ~/.bashrc

sudo apt-get install libsdl1.2-dev
sudo apt-get install libsdl-image1.2-dev
sudo apt install -y ros-$ROS_DISTRO-voxel-grid
sudo apt install -y ros-$ROS_DISTRO-costmap-converter
sudo apt install -y ros-$ROS_DISTRO-move-base-msgs
sudo apt install -y ros-$ROS_DISTRO-tf2-sensor-msgs
sudo apt install --upgrade ros-$ROS_DISTRO-octomap*
# sudo apt install -y ros-$ROS_DISTRO-mbf-costmap-core
# sudo apt install -y ros-$ROS_DISTRO-mbf-msgs
# sudo apt install -y ros-$ROS_DISTRO-teb-local-planner
