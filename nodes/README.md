# Controlling turtletim with 3D hand tracking
![turtle-demo.gif](../turtle-demo.gif)
These nodes allow you to control a [turtlesim](http://wiki.ros.org/turtlesim) using the output from the Seeed mgc3030 ROS driver. This folder contains the following files:
- `tracker_controller_basic.py`: Basic template for reading from the ROS driver and triggering commands.
- `tracker_controller_vel.py`: Further fleshes out the above to publish `cmd_vel` Twist messages, the standard topic for controlling robots in ROS.
- `tracker_controller_turtle.py`: Tuned to control the turtlesim.

## Setup
On the Pi / external computer ( remember to point ROS_MASTER_URI to the Pi and source the appropriate ROS distribution and workspaces )
```
sudo apt install ros-melodic-turtlesim

python tracker_controller_turtle.py # Or use rosrun
rosrun turtlesim turtlesim_node
```

## Parameters

The following parameters are used:
- `gesture_topic_name`: The topic for the gestures published by the ROS driver. Defaults to `mgc3030/gesture`
- `touch_topic_name`: The topic for the touch events published by the ROS driver. Defaults to `mgc3030/touch`
- `pos_topic_name`: The topic for the pos events published by the ROS driver. Defaults to `mgc3030/pos`
- `sensor_max_dist`: The maximum value for the sensor readings. Defaults to `0.1`
- `update_rate`: The rate of publishing / printing updates. Defaults to `10.0`
- `vel_topic`: The topic for `cmd_vel` topic. Defaults to `cmd_vel`, except `tracker_controller_turtle.py` which is `turtle1/cmd_vel`
- `max_vel`: The maximum velocity allowed for the vehicle. Defaults to `2.0`
- `max_angular_vel`: The maximum angular velocity, in radians, allowed for the vehicle. Defaults to `1.0`
