cmake_minimum_required(VERSION 3.0.2)
project(qt_ui)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  actionlib_msgs
  actionlib
  std_srvs
  sensor_msgs
  std_msgs
  tf2_ros
  laser_geometry
  pcl_conversions
  pcl_ros
  grid_map_ros
  custom_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

