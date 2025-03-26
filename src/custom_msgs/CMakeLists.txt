cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  actionlib_msgs
  actionlib
  std_srvs
  sensor_msgs
  std_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bbox.msg
  Bboxes.msg
  Drag.msg
  Click.msg
  Door.msg
  Ramp.msg
)

add_service_files(
  FILES
  Detect.srv
  Status.srv
  ShowGoal.srv
  GetPixel.srv
  UiCommand.srv
)

add_action_files(
  DIRECTORY action
  FILES
  ToDoor.action
  Detection.action
  Docking.action
  ClickPoint.action
  UpdateGoal.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs std_msgs geometry_msgs sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp actionlib actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)