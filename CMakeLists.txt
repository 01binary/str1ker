cmake_minimum_required(VERSION 3.0.2)
project(str1ker)

add_compile_options(-std=c++11 -Wno-psabi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  genmsg
  message_generation
  std_msgs
  controller_manager
  moveit_ros_robot_interaction
  moveit_core
  moveit_ros_planning
  pluginlib
  srdfdom
  urdf
  cmake_modules
)

find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")

add_message_files(
  DIRECTORY msg
  FILES
  Adc.msg
  Pwm.msg
  PwmChannel.msg
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
    genmsg
    cmake_modules
    message_runtime
    actionlib
    controller_manager
    moveit_ros_robot_interaction
    pluginlib
    moveit_core
    moveit_ros_planning
  DEPENDS
    EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(robot
  src/robot.cpp
  src/controllerFactory.cpp
  src/controller.cpp
  src/arm.cpp
  src/motor.cpp
  src/solenoid.cpp
  src/potentiometer.cpp
)

add_dependencies(
  robot ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(robot
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  -lrt
)

add_library(str1ker-ik
  src/ikPlugin.cpp
  src/ikSolver.cpp
)

target_link_libraries(str1ker-ik
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

add_library(str1ker-planner
  src/plannerPlugin.cpp
)

target_link_libraries(str1ker-planner
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

install(
  FILES
    description/planner.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
  RENAME
    str1ker-planner.xml
)

install(
  FILES
    description/ik.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
  RENAME
    str1ker-ik.xml
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
  FILES
    src/include/ikPlugin.h
  DESTINATION
    include
  RENAME
    str1ker-ikPlugin.h
)

install(
  FILES
    src/include/plannerPlugin.h
  DESTINATION
    include
  RENAME
    str1ker-plannerPlugin.h
)

install(
  TARGETS
    robot
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
