# Controlling turtlesim with 3D hand tracking
![turtle-demo.gif](../turtle-demo.gif)

These nodes allow you to control a ROS robots such as [turtlesim](http://wiki.ros.org/turtlesim) using the output from the Seeed mgc3030 ROS driver. This folder contains the following files:

- `tracker_controller_basic.py`: Basic template
- `tracker_controller_gesture_vel.py`: Sends cmd_vel commands, using gestures to set the travel mode
- `tracker_controller_gesture_turtle.py`: Tuned for turtle_sim
- `tracker_controller_point_vel.py`: Sends cmd_vel commands, based on position information only
- `tracker_controller_point_turtle.py`: Tuned for turtle_sim

We will focus on `tracker_controller_point_vel_turtle.py` as it offers the most practical controls.

By placing your hand over the shield at:
- A sharp backhand angle, you can drive the robot backwards
- A sharp "forward" angle, you can drive the robot fowards 
- A "chopping" pose to the right, you can rotate the robot clockwise
- A "chopping" pose to the left, you can rotate the robot counter-clockwise

The height of your hand above the shield will determine the magnitude of the motion.

To run the above, do the following if you did not set up to launch on startup:
```
roslaunch seeed_mgc3030_driver mgc3030_cmd_vel_turtle.launch

# On an external computer
sudo apt install ros-melodic-turtlesim
rosrun turtlesim turtlesim_node
```
