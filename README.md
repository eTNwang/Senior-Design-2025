﻿# Senior-Design-2025

## NUC
* Password: EthanMa
* User: Hue
* Hostname: hue-robot

## SSH
* ssh hue@hue-robot.local
* port could also be 2222
* IP may change

<h1>ROS2</h1>
Running ROS2 Jazzy

## Notes
* In ~/ros2_ws/src$ run 'colcon build' after changing the packages
* In ~/ros2_ws/src$ source install/local_setup.bash
* ros2 run <package> <node>

## Source
* Run this first in every terminal
* source /opt/ros/jazzy/setup.bash

## USB
* To find USB ports
* ls -l /dev/ttyACM*
* Made static versions
* ls -l /dev/ttyRobot*
* Enable (should be on by default)
* * sudo udevadm control --reload-rules
* * sudo udevadm trigger
* sudo nano /etc/udev/rules.d/99-usb-serial.rules

## Packages
https://github.com/jakedonnini/Hue_Ros_Packages


Offical packages are stored in this repo

## Teleop
* ros2 run teleop_twist_keyboard teleop_twist_keyboard
* Run this node as your keyboard input to the robot

