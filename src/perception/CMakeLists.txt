cmake_minimum_required(VERSION 3.0.2)
project(perception)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O2")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
  nodelet
)

# find_package(orocos_kdl REQUIRED)
# find_package(kdl_parser)
find_package( OpenCV REQUIRED )
find_package( PCL REQUIRED )
find_package( Eigen3 REQUIRED )

catkin_python_setup()


catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs custom_msgs actionlib nodelet
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  # ${orocus_kdl_INCLUDE_DIRS}
  # ${kdl_parser_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(camera_lib src/camera_detail.cpp)
target_link_libraries(camera_lib 
  ${catkin_LIBRARIES}
)

# add_executable(doorway_action src/doorway_action.cpp src/table_grid.cpp)
add_library(PerceptionNodelet src/perception_nodelet.cpp)
target_link_libraries(PerceptionNodelet camera_lib ${OpenCV_LIBS} ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(PerceptionNodelet ${catkin_EXPORTED_TARGETS})

add_executable(perception_node src/perception_node.cpp)
target_link_libraries(perception_node ${catkin_LIBRARIES})
add_dependencies(perception_node ${catkin_EXPORTED_TARGETS})

# add_library(TableGridNodelet src/tablegrid_nodelet.cpp)
# target_link_libraries(TableGridNodelet 
#   ${OpenCV_LIBS} 
#   ${PCL_LIBRARIES} 
#   ${catkin_LIBRARIES}
#   )
# add_dependencies(TableGridNodelet ${catkin_EXPORTED_TARGETS})

# add_executable(tablegrid_node src/tablegrid_node.cpp)
# target_link_libraries(tablegrid_node ${catkin_LIBRARIES})
# add_dependencies(tablegrid_node ${catkin_EXPORTED_TARGETS})

add_executable(visualizer_node src/visualizer.cpp)
target_link_libraries(visualizer_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(visualizer_node ${catkin_EXPORTED_TARGETS})

# target_link_libraries(doorway_action
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
#   ${orocus_kdl_LIBRARIES}
#   ${kdl_parser_LIBRARIES}
#   ${PCL_LIBRARIES}
#   camera_lib
# )

# catkin_install_python(PROGRAMS
#   scripts/ros_demo.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# catkin_install_python(PROGRAMS
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
