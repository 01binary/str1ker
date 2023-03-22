# Drumming Robot

Source code and parts for [Str1ker](https://www.01binary.us/projects/drumming-robot/) drumming robot. See [build log](https://hackaday.io/project/171607-drumming-robot) on Hackaday.io.

![body](./doc/readme/body.png)
![arms](./doc/readme/arms.png)
![wiring](./doc/readme/wiring.jpeg)

## Git

First-time git setup for a new machine:

```
git config --global user.name <your username>
git config --global user.email <your email>
git config --global credential.helper store
```

## Clone

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

git clone https://github.com/01binary/str1ker.git
git clone https://github.com/01binary/str1ker_moveit_config.git

cd ./str1ker
git sparse-checkout init --cone
git sparse-checkout set src launch description msg
```

## ROS

To setup ROS Noetic on a new Ubuntu Focal 20.04.x system, follow these instructions:

http://wiki.ros.org/noetic/Installation/Ubuntu

> ROS Noetic can only be installed on this distribution and version of Ubuntu, otherwise it has to be built from source (such as when installing on a Raspberry Pi or another dist)

## Install

```
cd ~/catkin_ws/src
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

## Build

```
cd ~/catkin_ws
catkin_make
```

## Setup

One-time setup after the first build.

```
catkin_make install
source /opt/ros/noetic/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Upload

The analog read and write duties are handled by an Arduino ROS node found in `/src/arduino`

To build it, first generate ROS message headers for Arduino:

```
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
sudo apt-get install ros-${ROS_DISTRO}-rosserial

rosrun rosserial_arduino make_libraries.py <Arduino libraries path>
```

Compile and upload `/src/analog.ino`.

The `robot.launch` file includes instructions to launch the Arduino node at robot startup. To launch manually for testing in isolation:

Run `roscore` if not already running:

```
roscore
```

Run the rosserial node (substitute `/dev/ttyACM0` for the port the Arduino is on):

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

## Launch

```
roslaunch str1ker robot.launch
```

## Launch in RViz

```
roslaunch str1ker_moveit_config demo.launch
```