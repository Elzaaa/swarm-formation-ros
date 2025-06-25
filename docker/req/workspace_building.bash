#!/bin/bash

EXEC_PATH=$PWD

mkdir -p ../noetic_ws/src && cd ../noetic_ws/src

git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
git clone -b master https://github.com/anybotics/grid_map.git

cd $EXEC_PATH

cp req/build.bash ../noetic_ws/
echo "!!!DONE!!!"
