cmake_minimum_required(VERSION 3.0.2)
project(str1ker)

add_compile_options(-std=c++11 -Wno-psabi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  angles
  genmsg
  message_generation
  std_msgs
  controller_manager
  control_toolbox
  moveit_core
  moveit_ros_robot_interaction
  moveit_ros_control_interface
  moveit_ros_planning_interface
  moveit_ros_planning
  pluginlib
  srdfdom
  urdf
  cmake_modules
)

find_package(Eigen3 REQUIRED)

add_message_files(
  DIRECTORY msg
  FILES
  VelocityCommand.msg
  PositionCommand.msg
  GripperCommand.msg
  StateFeedback.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    rospy
    tf2
    tf2_ros
    std_msgs
    sensor_msgs
    genmsg
    cmake_modules
    message_runtime
    actionlib
    controller_manager
    control_toolbox
    moveit_core
    moveit_ros_robot_interaction
    moveit_ros_control_interface
    moveit_ros_planning_interface
    moveit_ros_planning
    pluginlib
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(robot
  src/robot.cpp
  src/arm.cpp
)

add_dependencies(
  robot ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(robot
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  -lrt
)

add_executable(hardware
  src/hardware/hardware.cpp
)

add_dependencies(
  hardware ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(hardware
  ${catkin_LIBRARIES}
)

add_library(str1ker-ik
  src/plugins/inverseKinematicsPlugin.cpp
  src/plugins/inverseKinematicsSolver.cpp
)

target_link_libraries(str1ker-ik
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_library(str1ker-planner
  src/plugins/motionPlanningPlugin.cpp
)

target_link_libraries(str1ker-planner
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

install(
  TARGETS
    str1ker-ik
  LIBRARY
  DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS
    str1ker-planner
  LIBRARY
  DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  TARGETS
    robot
  RUNTIME
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    hardware
  RUNTIME
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
