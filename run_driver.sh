#!/bin/bash
set -e

# Source ROS and workspace
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Explicitly set ROS networking variables
export ROS_MASTER_URI=http://192.168.8.1:11311
export ROS_HOSTNAME=192.168.8.2
export ROS_IP=192.168.8.2

# Start Orbbec camera driver in background
roslaunch orbbec_camera femto_mega.launch camera_name:=orbbec/camera enable_colored_point_cloud:=true ordered_pc:=true 
# sleep 5  # optional delay

# Start rosbridge server # optional
# roslaunch rosbridge_server rosbridge_websocket.launch port:=9091


