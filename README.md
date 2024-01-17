# Pothole Detection System
## Summary 
The Pothole Inspection System is a software solution deployed on a LIMO mobile robot, enabling automated navigation and pothole detection on road surfaces. This system's primary objectives are to determine the total number of potholes, pinpoint their locations, and estimate their relative sizes. It encompasses navigation codes for robot control, a precise pothole detector, and a reporting mechanism for creating labeled maps, collectively facilitating efficient road inspection and reporting.
## Table of Contents
- [Features](#features)
- [Getting Started](#getting-started)
- [Installation](#installation)
- [Usage](#usage)
- [License](#license)
## Features
- Automatic pothole detection using computer vision
- Real-time reporting of pothole locations, size, number
- Start from other position
- Visualization of pothole data in RViz
## Installation

To install and run this project, follow these steps:
1. Install ROS:Installing ROS2 Humble desktop https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
2. Simulator Setup: https://github.com/LCAS/CMP9767_LIMO/wiki/Simulator-Setup
3. Creat Workspace: 
   ```bash
   mkdir ros2_ws
   cd ros2_ws
   mkdir src
   cd src
   git clone https://github.com/AMDPanda/Limo.git
   cd .. 
   colcon build
   
## Usage
4. Source the setup.bash for every terminal:
   ```bash
   source /opt/ros/humble/setup.bash
   source limo_ros2/install/setup.bash
   cd ros2_ws
   source install/setup.bash
5. Launch Gazebo Simulator(it could display the real simulation by replacing potholes_simple by potholes)
   ```bash
   ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/Limo/assignment_template/worlds/potholes.world
6. In a new Terminal, Launch Rviz((it could display the lower rate simulation by replacing 20e by 5) add the MarkerArray display topic:
    ```bash
    ros2 launch limo_navigation limo_navigation.launch.py use_sim_time:=true map:=src/Limo/assignment_template/maps/potholes_20mm.yaml params_file:=src/Limo/assignment_template/params/nav2_params.yaml
7. In a new Terminal, run report(map_flag= True if world = pothole_simple, map_flag = False if world = pothole). Remember to check the path of the image, or it may return empty graph
   ```bash
   ros2 run assignment_template report
8. In a new Terminal, run object_detector(map_flag= True if world = pothole_simple, map_flag = False if world = pothole)
   ```bash
   ros2 run assignment_template object_detector
9. In a new Terminal, run navigator
   ```bash
   ros2 run assignment_template example_nav_to_pose



