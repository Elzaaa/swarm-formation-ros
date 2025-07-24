# Swarm Navigation with Formation Control

This project is developed as part of the **ROS Programming elective course** at the Deggendorf Institute of Technology (THD).  
The goal is to implement swarm navigation with formation control using **ROS 1 (Noetic)** and mobile robots such as **TurtleBot3**.  
The system coordinates multiple robots to maintain formations (e.g., line or V-shape) during navigation while avoiding obstacles.

---
## Table of Contents

- [Overview](#overview)
- [Getting Started](#getting-started)
- [Project Structure](#project-structure) 

### Overview

To develop the swarm navigation system, we utilize a **leader-follower** approach. The leader robot navigates to a goal position while the follower robots maintain their formation relative to the leader. The system is designed to handle dynamic environments and obstacles, ensuring safe and efficient navigation.

The approach used to handle the navigation and formation control is based on the **potential field method**. The leader robot navigates towards a goal position while the follower robots adjust their positions to maintain the desired formation. The system also incorporates obstacle avoidance to ensure safe navigation. The Potential field method is implemented in the `swarm_navigation_controller.py` script, which is responsible for calculating the velocities of the robots based on their positions relative to the leader and the goal position.

For localization the robots inside the known environment, we use the **AMCL (Adaptive Monte Carlo Localization)** package. This package allows the robots to estimate their position and orientation based on sensor data and a known map of the environment.

For running the multiple robots in the same environment, we use the **Gazebo** simulator. The simulation environment is set up with multiple TurtleBot3 robots, and the `multi_robots_world.launch` file is used to launch the simulation with the desired number of robots. The control of each robot is done by using the Threads in Python, allowing the robots to run concurrently and independently while maintaining their formation.


This project currently uses:

> **Platform:** Ubuntu 22.04  
> **ROS Version:** Noetic  
> **Target Robots:** TurtleBot3 

---

## Getting Started

### Prerequisites

- [Docker](https://docs.docker.com/get-docker/)

### Installation and Usage

1. **Clone this repository:**<br>
    ```bash
    git clone git@github.com:Elzaaa/swarm-formation-ros.git
    cd swarm-formation-ros
    ```

2. **Install**<br>
    ```bash
    bash docker/install_docker.sh
    ```

3. **Build the Docker image:**<br>
    ```bash
    bash docker/build_docker.sh  .
    ```

4. **Run the container:**<br>
    ```bash
    bash docker/run_docker.sh
    ```

5. **Build the workspace:** <br>
    After building and running the image, it is necessary to build the work space `(noetic_ws)`:
    ```bash
    cd noetic_ws
    bash build.bash
    ```
6. **Open Gazebo with Launch File:**<br>
    ```bash
    roslaunch swarm_control multi_robots_world.launch
    ```

7. **Open another terminal window and access the running container:**<br>
    ```bash
    bash docker/into_docker.sh
    ```

7. **Run the Navigation Controller script:**<br>
    ```bash
    cd noetic_ws/src
    python3 swarm_navigation_controller.py
    ```
8. **Start the navigation:**<br>
    The launch file on the step 6 should open an `Rviz` window. There use the `2D Pose Estimate` to indicate the leader current position and the `2D Navigation Goal` to set the goal position. 

---


## Project Structure

```
<swarm-formation-ros>/
├── noetic_ws/ — ROS workspace
├── docker/ — Dockerfile and related scripts
├── scripts/ - Specific purpose scripts
└── README.md
```

