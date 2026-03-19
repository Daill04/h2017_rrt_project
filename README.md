# Doosan H2017 Manipulator: Motion Planning & Obstacle Avoidance

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue.svg)
![C++](https://img.shields.io/badge/C++-14%2F17-blue.svg)
![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Enabled-orange.svg)

## 📌 Overview
This repository contains a ROS 2 (Humble) based motion planning framework for the **Doosan H2017** 6-DOF industrial collaborative robot. It leverages MoveIt 2 and the OMPL library to perform collision-free trajectory generation in cluttered environments. 

The project strictly focuses on industrial-grade trajectory execution, incorporating **Time-Optimal Trajectory Generation (TOTG)** to ensure smooth, high-frequency motor control and eliminate mechanical jitter during path execution.

## 🎥 Demonstration
[rrt_doosan.webm](https://github.com/user-attachments/assets/5ed21e00-23be-43b4-a83b-239081638a35)


## 🚀 Core Features
1. **`basic_motion_node`**: 
   * Demonstrates Cartesian space (XYZ) Inverse Kinematics (IK) solving.
   * High-density trajectory interpolation for smooth Point-to-Point (P2P) execution.
2. **`rrt_node`**:
   * Dynamic collision object injection via `PlanningSceneInterface`.
   * Asymptotically optimal obstacle avoidance using the **RRT*** algorithm.
   * Trajectory smoothing and velocity/acceleration scaling for physical safety constraints.

## 🛠️ Prerequisites
* Ubuntu 22.04 LTS
* [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)
* [MoveIt 2](https://moveit.ros.org/)
* [Doosan Robotics ROS 2 Package](https://github.com/doosan-robotics/doosan-robot2) (`humble-devel` branch)

## 🏗️ Build Instructions

Clone this repository into your ROS 2 workspace `src` directory:
```bash
cd ~/ros2_ws/src
git clone [https://github.com/YourUsername/h2017_rrt_project.git](https://github.com/YourUsername/h2017_rrt_project.git)
cd ~/ros2_ws
colcon build --packages-select h2017_rrt_project
source install/setup.bash
