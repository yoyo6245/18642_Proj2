#!/bin/bash
maze_num=""
echo "$1"
maze_num=m$1.maze
echo $maze_num
./build_run_turtle.sh $maze_num
