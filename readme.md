# Drumming Robot

Source code and parts for [Str1ker](https://www.01binary.us/projects/drumming-robot/) drumming robot. See [build log](https://hackaday.io/project/171607-drumming-robot) on Hackaday.io.

![body](./doc/readme/body.png)
![arms](./doc/readme/arms.png)
![wiring](./doc/readme/wiring.jpeg)

## Clone

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

git clone https://github.com/01binary/drummingrobot.git

git sparse-checkout init --cone
git sparse-checkout set src/code launch msg
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
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
sudo apt-get install ros-${ROS_DISTRO}-rosserial

rosrun rosserial_arduino make_libraries.py .
```

Upload the `adc.ino` to Arduino Micro

Run rosserial node to get ADC inputs published on `/robot/adc`:

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

## Run this package

```
source devel/setup.bash
roslaunch str1ker robot.launch
```
