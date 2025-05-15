#!/bin/bash

docker run -it --rm --name ros-humble-echo \
  --net=host \
  --privileged \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v ~/swarm-formation-ros:/home/ros-user/swarm-formation-ros \
  ros-humble-img
