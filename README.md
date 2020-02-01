# Seeed mgc3130 ROS driver
![demo.gif](demo.gif)

ROS interface for data output from the [Seeed mgc3130](http://wiki.seeedstudio.com/3D-Gesture-Tracking-Shield-for-Raspberry-Pi-MGC3130/), [github](https://github.com/Seeed-Studio/Seeed_mgc3x30.git). plugged into a Raspberry Pi.

## Setup instructions
1. Set up a Raspberry Pi with a ROS1 distribution. You can folllow these [instructions](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi). Another option is to use Ubiquity Robotics [Pi Image](https://downloads.ubiquityrobotics.com/pi.html), and run the following:
```
sudo systemctl disable pifi.service
sudo systemctl disable magni-base.service
``` 
This option has the benefit of having a `roscore` automatically launch at boot using systemd.

2. Clone this repository into a workspace:
```
cd ~
mkdir -p hand_tracking_ws/src
cd hand_tracking_ws/src
git clone https://github.com/cnboonhan94/seeed_mgc3030_driver.git
```
3. Run the initialization script from the ROS package folder to install dependencies:
```
cd ~/hand_tracking_ws/src/seeed_mgc3030_driver
./tools/init_sh
```

3. Build and launch! I use [python-catkin-tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).
```
cd ~/hand_tracking_ws
catkin build 
source devel/setup.bash
roslaunch seeed_mgc3030_driver mgc3030.launch
rviz
```

4. To force a recalibration, you can do:
```
rostopic pub /mgc3030/reset std_msgs/String "type anything here"
```

## Parameters
- `pos_topic_name`: String, specifies the topic to publish PointStamped messages of the sensor readings. Defaults to `pos`.
- `touch_topic_name`: String, specifies the type of touch input the sensor is reading. Defaults to `touch`.
- `reset_topic_name`: String, publish any message to this topic to force a recalibration of the sensors. This is useful when the environment changes ( such as an additional layer of material being added over the shield ) and the readings go haywire. Defaults to `reset`.
- `sensor_max_dist`: The maximum value that the sensor can read. Can be set to "amplify" the readings from the sensor. A good value for visualization is 5.0. Defaults to `0.1`.
