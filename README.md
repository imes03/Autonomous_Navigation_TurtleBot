
# Gazebo-Maze-Exploration-with-Turtlebot3
Gazebo-ROS2 Humble autonomous exploration of TurtleBot3 using SLAM Toolbox, Nav2, RRT-frontier based exploration and AprilTag detection.
by: Anchia Wilmer and Mehrabi Bahram.
    
# TurtleBot3: Autonomous Maze Exploration with ROS2 and Gazebo

## About

This project implements an autonomous exploration framework for the **TurtleBot3 Burger with RGB camera** using **ROS 2 Humble** and **Gazebo**. The system combines **SLAM Toolbox**, **Navigation2**, **frontier-based exploration using Rapidly-exploring Random Trees (RRT)**, and **AprilTag detection** to autonomously explore unknown indoor environments.

The robot incrementally builds an occupancy grid map, detects frontiers, selects the next exploration target using RRT, navigates autonomously using Navigation2, and simultaneously detects AprilTags using an RGB camera. The project was developed as part of a compulsory elective course "FWP:ROS" in a Master's program in Mechatronics and Cyber-Physical Systems.

---

## Features

- ROS 2 Humble
- TurtleBot3 Burger with RGB Camera
- Gazebo Simulation
- SLAM Toolbox
- Navigation2
- Frontier-Based Exploration
- Rapidly-exploring Random Trees (RRT)
- AprilTag Detection
- RViz2 Visualization

---

# Project Structure

```
Autonomous_Navigation_TurtleBot/
│
├── src/
│   └── autonomous_tb3/
│       ├── autonomous_tb3/
│       │   ├── explorer_node.py
│       │   ├── maze_solver.py
│       │   ├── camera_viewer.py
│       │   ├── apriltag_overlay.py
│       │   ├── apriltag_text_marker.py
│       │   └── occupancy_grid_pub.py
│       │
│       ├── launch/
│       │   ├── maze_navigation.launch.py
│       │   ├── mapping.launch.py
│       │   └── tb3_world_navigation.launch.py
│       │
│       ├── config/
│       │   ├── slam_toolbox_params.yaml
│       │   ├── tb3_nav_params.yaml
│       │   ├── apriltag.yaml
│       │   └── maze.yaml
│       │
│       ├── urdf/
│       ├── world/
│       ├── meshes/
│       └── package.xml
│
└── README.md
```

---

# Running the Simulation

## Terminal 1

Launch Gazebo, TurtleBot3, Navigation2 and SLAM.

```bash
source install/setup.bash

export TURTLEBOT3_MODEL=burger_cam

ros2 launch autonomous_tb3 maze_navigation.launch.py
```

---

## Terminal 2

### Set Gazebo Model Path

```bash
export GAZEBO_MODEL_PATH=/ws_slam/src/Autonomous_Navigation_TurtleBot/src/autonomous_tb3/world/tags
```

### Spawn AprilTag

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity test1 \
  -file /ws_slam/src/Autonomous_Navigation_TurtleBot/src/autonomous_tb3/world/tags/apriltag_working/model.sdf \
  -x -11.05 \
  -y 3 \
  -z 0.5
```

### Optional Additional AprilTag

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity test3 \
  -file /ws_slam/src/Autonomous_Navigation_TurtleBot/src/autonomous_tb3/world/tags/apriltag_working/model.sdf \
  -x 3 \
  -y 6 \
  -z 0.5
```

### Optional Small Maze

```bash
ros2 run autonomous_tb3 sdf_spawner \
/ws_slam/install/autonomous_tb3/share/autonomous_tb3/world/small_maze/model.sdf \
small_maze -8.0 8.5
```

### Optional AprilTag for Small Maze

```bash
ros2 run gazebo_ros spawn_entity.py \
  -entity test5 \
  -file /ws_slam/src/Autonomous_Navigation_TurtleBot/src/autonomous_tb3/world/tags/apriltag_working/model.sdf \
  -x -10.25 \
  -y 8.4 \
  -z 0.3 \
  -Y 1.5708
```

### Launch AprilTag Detection

```bash
ros2 run apriltag_ros apriltag_node \
  --ros-args \
  -r image_rect:=/camera/image_raw \
  -r camera_info:=/camera/camera_info \
  --params-file \
  /ws_slam/src/Autonomous_Navigation_TurtleBot/src/autonomous_tb3/config/apriltag.yaml
```

---

## Terminal 3

Run the autonomous exploration node.

```bash
source install/setup.bash

ros2 run autonomous_tb3 explorer_node
```

---


## Demo Video
🎥 [Click here to watch the demo](https://github.com/Emaaaad/autonomous_turtle_bot3/blob/main/Demo.mp4)



# Software

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo
- Navigation2
- SLAM Toolbox
- RViz2
- AprilTag ROS
- Python

---

# Repository

This repository is based on the open-source project:

https://github.com/Razavi1993/Autonomous_Navigation_TurtleBot

and contains additional developments including:

- Frontier-based exploration improvements
- RRT frontier selection
- AprilTag integration
- Autonomous exploration node
- TurtleBot3 Burger camera simulation

Please also check for the packages, run:

```bash
apt-get update && apt-get install -y ros-humble-slam-toolbox \
                   ros-humble-turtlebot3 \
                   ros-humble-turtlebot3-simulations \
```



