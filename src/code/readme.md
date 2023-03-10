# str1ker

This directory includes ROS source code for the drumming robot that should be cloned into `~/catkin_ws/src` and built with `catkin_make` after installing/building dependencies.

## Clone

```
cd ~/catkin_was/src

git clone https://github.com/01binary/drummingrobot.git

git sparse-checkout init --cone
git sparse-checkout set src/code launch
```

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
source ~/catkin_ws/devel/setup.bash
```

## Upload

The analog-to-digital controller runs on Arduino Micro and communicates back through rosserial library.

Generate message headers:

```
rosserial_arduino make_libraries.py .
```

Upload the `adc.ino` to Arduino Micro

Run rosserial node to get ADC inputs published on `/robot/adc`:

```
rosrun rosserial_python serial_node.py /dev/ttyUSB0
```

## Run this package

```
source devel/setup.bash
roslaunch str1ker robot.launch
```
