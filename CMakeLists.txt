cmake_minimum_required(VERSION 3.0.2)
project(str1ker)

add_compile_options(-std=c++11 -Wno-psabi)

add_definitions(-DDEBUG)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
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

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs dynamixel_workbench_msgs trajectory_msgs dynamixel_workbench_toolbox cmake_modules
  DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(robot
  src/code/main.cpp
  src/code/robot.cpp
  src/code/controllerFactory.cpp
  src/code/controller.cpp
  src/code/arm.cpp
  src/code/pwmServo.cpp
  src/code/dynamixelPro.cpp
  src/code/linear.cpp
  src/code/solenoid.cpp
  src/code/potentiometer.cpp
  src/code/mcp3008.cpp
  src/code/ads1115.cpp
  src/code/pi16adc.cpp
  src/code/arduinoMicro.cpp
)

add_dependencies(robot ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${YAML_CPP_LIBRARIES} -lpigpio)

install(
  TARGETS robot
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

file(
  COPY ${PROJECT_SOURCE_DIR}/build/post_build.sh
  DESTINATION ${PROJECT_SOURCE_DIR}
  FILE_PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

add_custom_command(
  TARGET robot
  POST_BUILD COMMAND ${PROJECT_SOURCE_DIR}/post_build.sh
)
