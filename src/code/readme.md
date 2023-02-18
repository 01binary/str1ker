# str1ker

This directory includes ROS source code for the drumming robot that should be cloned into `~/catkin_ws/src` and built with `catkin_make` after installing/building dependencies.

## Dependencies

|Dependency|Install|
|-|-|
|roscpp|Included with ros|
|sensor_msgs|Included with ros|
|geometry_msgs|Included with ros|
|trajectory_msgs|Included with ros|
|dynamixel_workbench_msgs|Clone [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) and build from source (not availble for Melodic at the time of writing)|
|dynamixel_workbench_toolbox|Clone [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK) and build from source (not availble for Melodic at the time of writing)|
|cmake_modules|Included with cmake|
|yaml-cpp|Included with ros|
|eigen|Included with dynamixel SDK|

## Clone

If you are cloning on raspberri pi, it's recommended to limit changes being tracked to this directory (`src/code`). To do that, use `sparse-checkout` command available if you build the latest `git` from source on Raspberry Pi. At the time of writing, there is no git package available for Raspbian that supports sparse-checkout, which is why it needs to be built.

```
cd ~/catkin_was/src

git clone https://github.com/01binary/drummingrobot.git

git sparse-checkout init --cone

git sparse-checkout set src/code launch
```

## Build

```
cd ~/catkin_ws
catkin_make
```

## Run this package

```
source devel/setup.bash
roslaunch str1ker robot.launch
```
