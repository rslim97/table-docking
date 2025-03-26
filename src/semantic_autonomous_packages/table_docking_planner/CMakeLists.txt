cmake_minimum_required(VERSION 3.0.2)
project(table_docking_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  custom_msgs
  pcl_ros
  pcl_conversions
  actionlib
  actionlib_msgs
  grid_map_ros
  grid_map_costmap_2d
  tf2_ros
  costmap_2d
  geometry_msgs
  nav_msgs
  dwa_local_planner
  teb_local_planner
  navfn
  carrot_planner
  message_filters
)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES table_docking_lib
 CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(planner_node src/planner.cpp)

add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(planner_node
  ${catkin_LIBRARIES}
)
