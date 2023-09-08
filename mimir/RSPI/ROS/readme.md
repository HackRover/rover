THIS IS TESTING ALL CODE HAVE NOT BE REALLY TO GO

How to install:
Install ROS Noetic on your Raspberry Pi 3B+ if you haven't already. You can follow the official installation guide for Raspberry Pi: http://wiki.ros.org/noetic/Installation/Ubuntu

Create a ROS package (if you haven't already) for your LED control project. Replace your_package_name with your desired package name:
```bash
cd ~/catkin_ws/src
catkin_create_pkg your_package_name rospy std_msgs
```
Create a Python script for controlling the LED. You can use the RPi.GPIO library to toggle GPIO pin 12. Install it if you haven't already:
```python
pip install RPi.GPIO
```
Create a Python script, e.g., led_control.py inside your ROS package's src folder:

Make sure to modify the ROS package's CMakeLists.txt and package.xml files to include the necessary dependencies and build instructions.

Build your ROS package:
```bash
cd ~/catkin_ws
catkin_make
```
Before running your ROS node, make sure you've connected everything to the right GPIO pins, with an appropriate resistor and power source.

Run your ROS node:
```bash
rosrun your_package_name led_control.py
```

For the File 
