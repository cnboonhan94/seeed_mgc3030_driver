#!/bin/bash
# Populate the following if running roscore on another device
# export ROS_MASTER_URI=10.42.0.1:11311
# export ROS_IP=10.42.0.248
# Remember to add an entry for the master in /etc/hosts

# Script to launch on boot
source /opt/ros/kinetic/setup.bash
source $HOME/seeed_ws/devel/setup.bash
roslaunch seeed_mgc3030_driver mgc3030_cmd_vel.launch
