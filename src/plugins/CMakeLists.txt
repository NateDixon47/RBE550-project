cmake_minimum_required(VERSION 3.0.2)
project(simple_move)

# Find Gazebo
find_package(gazebo REQUIRED)

# Include directories
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Link directories
link_directories(${GAZEBO_LIBRARY_DIRS})

# Set compiler flags
set(CMAKE_CXX_STANDARD 17) # Use C++17 or adjust if necessary
add_definitions(${GAZEBO_CXX_FLAGS})

# Define the plugin library
add_library(simple_move SHARED simple_move.cpp)

# Link the Gazebo libraries
target_link_libraries(simple_move ${GAZEBO_LIBRARIES})
