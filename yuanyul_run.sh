#!/bin/bash
source ~/catkin_ws/devel/setup.bash
maze_num=""
echo "$1"
maze_num=m$1.maze
echo $maze_num
gnome-terminal --command="bash -c 'cd ~/catkin_ws;catkin_make ece642rtle_turn_monitor;devel/setup.bash; cd src/ece642rtle/monitors; ./run_642_monitors.sh ece642rtle_turn_monitor; $SHELL'"
./build_run_turtle.sh $maze_num
