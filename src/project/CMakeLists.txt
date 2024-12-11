cmake_minimum_required(VERSION 2.8.3)
project(project)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_core
  geometry_msgs
  nav_msgs
  pluginlib  # For registering the global planner
  costmap_2d # For costmap integration
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs nav_core geometry_msgs nav_msgs costmap_2d pluginlib
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add C++ executable for other nodes (if any)
add_executable(key_control src/key_control.cpp)
target_link_libraries(key_control ${catkin_LIBRARIES})

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/global_planner.py  # Your Python global planner script
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install map and parameter files
install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)

install(DIRECTORY param/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/param
)

## Install RViz config files
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

## Install plugin description file for the custom global planner
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
