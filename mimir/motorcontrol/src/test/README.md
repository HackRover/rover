
# Setup

You do not need to clone until "Project Setup"

## How to install ROS
This is only for setups that haven't touched ROS at all before. Requires Ubuntu 20.04 and the for repositories to allow "restricted," "universe," and "multiverse."

Run the following in terminal
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
```

## Enviroment Setup + Dependancies
```
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
sudo apt-get install cmake python-catkin-pkg python-empy python-nose python-setuptools libgtest-dev build-essential
mkdir build && cd build && cmake ../ && make && sudo make install
```

## Project Setup

Clone the repository into catkin_ws/src

To make node executable:
```
cd catkin_ws/src
chmod +x test/src/publisher.py
```

## Running nodes

In three seperete terminals, run 

Terminal Main
```
roscore
```
Terminal 2
```
rosrun test publisher.py
```
Terminal 3
```
rosnode list
rostopic list | grep saw _
rostopic echo /say-hello
```

Expected output:
Terminal 2 publishes a topic
Terminal 3 echos it and outputs 'data: "Hey!"

