cmake_minimum_required(VERSION 3.0.2)
project(str1ker)

add_compile_options(-std=c++11 -Wno-psabi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  angles
  genmsg
  message_generation
  std_msgs
  tf2_geometry_msgs
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
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs)

catkin_package(
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
  DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(
  robot
  src/code/main.cpp
  src/code/robot.cpp
  src/code/controllerFactory.cpp
  src/code/controller.cpp
  src/code/arm.cpp
  src/code/pwmServo.cpp
  src/code/solenoid.cpp
  src/code/potentiometer.cpp
)

add_dependencies(
  robot ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  robot
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  -lpigpiod_if2
  -lrt
)

install(
  TARGETS robot
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
