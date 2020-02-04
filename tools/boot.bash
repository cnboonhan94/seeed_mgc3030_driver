#!/bin/bash
source /opt/ros/kinetic/setup.bash
source $HOME/seeed_ws/devel/setup.bash
roslaunch seeed_mgc3030_driver mgc3030_cmd_vel_turtle.launch
