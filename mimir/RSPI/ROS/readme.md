<p>
THIS IS TESTING ALL CODE HAVE NOT BE REALLY TO GO
FIXME: Have not yet tested on the RSPI. I need some people who have RSPI that have ROS to run and tell me what error code.
FIXME: THE version so far I have written is in the TEST_PWM_RSPI.py However, using TEST_led_contorl.py to test the board will be the first thing to do so far.
</p>
<h1><b>
Welcome to ROS for RSPI contorl mortor
</b></h1>
<p>
This ROS package is designed for ROS noetic to control the motor when connected to Ubuntu 20.0.xx on RSPI.
When it runs it should be a Subscriber to the Publisher. 
However, it also will be Publisher when it needs to send the state of the motor back to the higher level.
</p>

<p>
Under the How to install we have used the example file TEST_led_contorl.py as an example.
For TEST_led_contorl.py make sure you have an LED connected to Pin 12 on the RSPI
<br>
<b>How to install:
</b></p>
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
Example is TEST_led_contorl.py

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

For the File TEST_led_contorl.py, some controls messages that can be used:
```bash
rostopic pub /led_control std_msgs/Bool "data: true"  # Turn on the LED
rostopic pub /led_control std_msgs/Bool "data: false" # Turn off the LED
```
