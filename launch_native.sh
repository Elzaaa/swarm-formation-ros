#!/bin/bash

echo "=== 🐝 Swarm Navigation Launcher ==="

echo "[1/5] Sourcing ROS 2 Humble..."
source /opt/ros/humble/setup.bash

echo "[2/5] Changing to ros_ws..."
cd ~/swarm-formation-ros/ros_ws || { echo "Failed to enter ros_ws"; exit 1; }

echo "[3/5] Building workspace..."
colcon build --event-handlers console_cohesion+
source install/setup.bash

echo "[4/5] Launching control node (my_node)..."
gnome-terminal --title="Swarm Control" -- bash -c "source install/setup.bash; ros2 run swarm_control my_node --ros-args -p robot_id:=1; exec bash"

echo "[5/5] Launching listener node..."
gnome-terminal --title="Listener Node" -- bash -c "source install/setup.bash; ros2 run listener listener_node; exec bash"

echo "🐝 Swarm nodes launched in separate terminals."
