# Drumming Robot

Source code and parts for [Str1ker](https://www.01binary.us/projects/drumming-robot/) drumming robot. See [build log](https://hackaday.io/project/171607-drumming-robot) on Hackaday.io.

![body](./doc/readme/body.png)
![arms](./doc/readme/arms.png)
![wiring](./doc/readme/wiring.jpeg)

## System Requirements

### Hardware

**Minimum**: [Raspberry Pi 4B 8GB](https://www.amazon.com/Raspberry-Pi-Computer-Suitable-Workstation/dp/B0899VXM8F) + [Arduino Micro](https://www.amazon.com/Arduino-Micro-Headers-A000053-Controller/dp/B00AFY2S56)

**Recommended**: [Latte Panda 3 Delta](https://www.amazon.com/LattePanda-Delta-864-Pocket-Sized-Computer/dp/B0BB7CY51B)

### Software

The Robot Operating System (ROS) only runs on Linux. ROS Noetic packages are only available for Ubuntu Focal `20.04.x`.

Running on Raspbian requires [building ROS from source](https://varhowto.com/install-ros-noetic-raspberry-pi-4/).

See [Installing from source](http://wiki.ros.org/noetic/Installation/Source) for more information.

## Source Control

First-time git setup on Ubuntu:

```
git config --global user.name <your username>
git config --global user.email <your email>
git config --global credential.helper store
```

Clone the projects. When asked for password, paste your [Personal Access Token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token).

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

git clone https://github.com/01binary/str1ker.git
git clone https://github.com/01binary/str1ker_moveit_config.git

cd ./str1ker
git sparse-checkout init --cone
git sparse-checkout set src launch description msg
```

## Install Packages

```
cd ~/catkin_ws/src
rosdep install -y --from-paths . --ignore-src --rosdistro noetic
```

## Build

```
cd ~/catkin_ws
catkin_make
```

## Export

One-time setup after the first build.

```
catkin_make install
source /opt/ros/noetic/setup.bash

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Upload

The analog read and write duties are handled by an Arduino ROS node in `src/analog.ino`.

To build it, first generate ROS message headers for Arduino:

```
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
sudo apt-get install ros-${ROS_DISTRO}-rosserial

rosrun rosserial_arduino make_libraries.py <Arduino libraries path>
```

Compile and upload the ROS node. The default launch configuration in `robot.launch` will connect to `/dev/ttyACM0` automatically.

To launch manually for testing in isolation:

Run `roscore` if not already running:

```
roscore
```

Run the `rosserial` node (substitute `/dev/ttyACM0` for the port the Arduino is on):

```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

## Launch

To launch the robot on the real hardware:

```
roslaunch str1ker robot.launch
```

## Launch in RViz

To launch the robot on simulated hardware:

```
roslaunch str1ker_moveit_config demo.launch
```

## Configure MoveIt

Follow [MoveIt installation instructions](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html) to install and build MoveIt on ROS Noetic.

```
roslaunch moveit_setup_assistant setup_assistant.launch
```