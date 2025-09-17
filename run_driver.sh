#!/bin/bash
set -e

# Source environments
source /opt/ros/noetic/setup.bash
source /root/catkin_ws/devel/setup.bash

# Keep restarting driver if it crashes
while true; do
    echo "Starting Orbbec ROS driver..."
    roslaunch orbbec_camera femto_bolt.launch \
        camera_name:=orbbec/camera \
        enable_colored_point_cloud:=true \
        ordered_pc:=true
    echo "Driver exited, restarting in 5s..."
    sleep 5
done
