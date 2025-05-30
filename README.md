# Swarm Navigation with Formation Control

This project is developed as part of the **ROS Programming elective course** at the Deggendorf Institute of Technology (THD).  
The goal is to implement swarm navigation with formation control using **ROS 1 (Noetic)** and mobile robots such as **TurtleBot3**.  
The system coordinates multiple robots to maintain formations (e.g., line or V-shape) during navigation while avoiding obstacles.

---

### 🐳 Docker Scripts

Here are some scripts that simplify working with this project.  
The setup is based on [TCC-ROS/ros-humble](https://github.com/TCC-ROS/ros-humble](https://github.com/TCC-ROS/ros-noetic), originally created for the ROS Programming Course at THD.

> **Platform:** Ubuntu 22.04  
> **ROS Version:** Noetic  
> **Target Robots:** TurtleBot3 or simulated agents

---

#### 🚀 Quick Start

Install Docker (with NVIDIA support if available):

```bash
bash install_docker.sh -n     # (Re)install Docker
bash build_docker.sh -n       # Build Docker container
bash run_docker.sh -n         # Run Docker container
```

---

#### 🐅 Access the running container:


```bash
bash into_docker.sh
```

