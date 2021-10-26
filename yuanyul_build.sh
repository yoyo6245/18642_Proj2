#!/bin/bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
catkin_make ece642rtle_student
catkin_make ece642rtle_node
cd ~/catkin_ws/src/ece642rtle
