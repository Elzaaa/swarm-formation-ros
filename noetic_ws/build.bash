#!/bin/bash

cd /noetic_ws \
     && . /opt/ros/noetic/setup.sh \
     && catkin_make
   
exec bash

