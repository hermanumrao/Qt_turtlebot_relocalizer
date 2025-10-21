#!/bin/bash

sudo apt update
sudo apt install libunwind-dev
sudo apt install ros-humble-cartographer ros-humble-cartographer-ros
sudo apt install ros-humble-nav2-bringup ros-humble-navigation2 ros-humble-nav2-msgs
sudo apt install libxcb-xinerama0 libxcb-xinerama0-dev libxcb-cursor0 libxcb-cursor-dev libxkbcommon-x11-0
sudo apt install libxcb1 libx11-xcb1 libxrender1 libxrandr2 libxfixes3 libx11-dev

pip install -r requirements.txt

mkdir -p ~/ros2_ws/src/
mv random_localizer ~/ros2_ws/src/
mv maps ~/maps/

cd ~/ros2_ws/
colcon build
cd 
