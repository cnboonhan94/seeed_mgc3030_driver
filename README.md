# Seeed mgc3130 ROS driver
ROS interface for data output from the [Seeed mgc3130](http://wiki.seeedstudio.com/3D-Gesture-Tracking-Shield-for-Raspberry-Pi-MGC3130/), [github](https://github.com/Seeed-Studio/Seeed_mgc3x30.git). plugged into a Raspberry Pi.

## Setup instructions
1. Raspberry Pi with a ROS1 distribution. You can folllow these [instructions](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi). Another option is to use Ubiquity Robotics [Pi Image](https://downloads.ubiquityrobotics.com/pi.html), and run the following:
```
sudo systemctl disable pifi.service
sudo systemctl disable magni-base.service
```

2. Run the initialization script from the ROS package folder:
```
cd (seeed_mgc3130_driver_path)
./tools/init_sh
```
