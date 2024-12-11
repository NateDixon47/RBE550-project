# RBE550 Project

This repository contains the implementation of a global and local planning system for autonomous robot navigation in dynamic environments. The project leverages ROS (Robot Operating System) for integrating global path planning using a custom A* algorithm and local obstacle avoidance using TrajectoryPlannerROS.

## Features

- **Global Path Planning**: A* algorithm adapted for continuous space, ensuring smooth and optimized paths.
- **Local Obstacle Avoidance**: Real-time obstacle detection and avoidance using Lidar data.
- **Dynamic Environment Handling**: Integration of global and local planners for robust navigation.

## Prerequisites

Before building the project, ensure the following dependencies are installed:

- **ROS**: ROS Noetic (or a compatible version)
- **Gazebo**: For the simulation environment
- **CMake**: Version 3.10 or later
- **catkin_tools**: For building the project
- Other ROS packages:
  - `costmap_2d`
  - `nav_msgs`
  - `geometry_msgs`
  - `tf`
  - `sensor_msgs`

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/NateDixon47/RBE550-project.git
   cd RBE550-project/project_ws

2. Build the project using 'catkin_make'
3. Source the workspace using 'source devel/setup.bash'
4. To launch the simulation, use 'roslaunch project final.launch'. This will bring up the robot and world in Gazebo and Rviz. Use the nav2goal function in rviz to send goal positions to the robot. Add obstacles in Gazebo to test the obstacle avoidance. 
