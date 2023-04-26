# rover
A repository for all rover software.

## Webserver
Start with running 
```
roscore
```
in a new terminal to start ROS.

Then in a new terminal, navigate to the websever directory in the Jetson Nano and run the shell script
```
sh weblaunch.sh
```

Then in a new terminal launch the ROS serial server:
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```
That should be all the commands neccessary to start the server to interact with ROS and the Arduino as a motor controller.
You can also run to preview active ROS topics
```
rostopic list
```
And also run the following to check out the /cmd_vel ROS topic which is where the velocity values of ROS are provided.
```
rostopic echo /cmd_vel
```
Webserver folder contains the website that is hosted on the Jetson nano that displays the real time LIDAR view. 
Run the shell script to launch the webserver.

Here is the output:
![live view](https://github.com/)
