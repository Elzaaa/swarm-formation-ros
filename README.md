# Swarm Navigation with Formation Control

This project is developed as part of the **ROS Programming elective course** at the Deggendorf Institute of Technology (THD).  
The goal is to implement swarm navigation with formation control using ROS 2 (Humble) and the Moveti2 platform.  
The system aims to coordinate multiple mobile robots while maintaining a predefined formation, such as a line or V-shape, during navigation.

---

### 🐳 Docker Scripts

Here are some scripts that simplify working with this project.  
The setup is based on [TCC-ROS/ros-humble](https://github.com/TCC-ROS/ros-humble), originally created for the ROS Programming Course at THD.

> **Platform:** Ubuntu 22.04  
> **ROS2 Version:** Humble  
> **Target Robot:** Moveti2

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
