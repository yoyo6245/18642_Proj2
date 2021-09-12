#!/bin/bash

# Will kill all processes that are passed as arguments
# Used with trap to guarantee that CTRL+C on this script kills all BG processes
kill_processes() {
    for p in "$@"; do
	if [[ -z $(ps -p $p > /dev/null) ]]; then
	    echo "Killing process $p"
	    kill $p
	    sleep 1
	fi
    done
    echo "killed all processes, exiting"
    exit 0
}

# Arguments checking
maze_file=""
if [[ $# -gt 1 ]]; then
    echo "Only one argument allowed: build_run_turtle.sh [-h|--help] [MAZEFILE_NAME]"
    exit 1
fi

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo "Usage: ./build_run_turtle.sh [-h|--help] [MAZEFILE_NAME]"
    exit 2
fi

# Locate the ece642rtle directory
turtledir=`rospack find ece642rtle`
if [ -z "$turtledir" ]; then
    echo "Cannot locate package ece642rtle."
    echo "Did you run catkin_make (with no arguments) in the"
    echo " Catkin workspace before proceeding?"
    exit 1
fi

# Catkin WS is two directories above package directory
cd "$turtledir/../.."
echo "Working from catkin workspace $(pwd)"
echo ""

# Locate mazefile, if any
if [[ $# -eq 1 ]]; then
    if [ "${1: -5}" != ".maze" ]; then
	echo "Argument must be a mazefile. You provided: $1"
	exit 1
    fi
    maze_file=$1
    # Check that maze file is not empty
    if [ ! -s "$turtledir/$maze_file" ]; then
	echo "Maze file $turtledir/$maze_file does not exist."
	echo "Be sure you are only providing the name of the mazefile, and not the path."
	echo "Example: m1.maze rather than src/ece642rtle/m1.maze"
	exit 1
    fi
fi

# Build the student node
catkin_make ece642rtle_student
if [ $? -ne 0 ]; then
    echo "catkin_make did not succeed, exiting script"
    exit 1
fi

# So ROS can find everything
source devel/setup.bash

# Run roscore if it is not already running
ROSCORE_PID=""
if [[ -z $(pgrep roscore) ]]; then
    roscore&
    ROSCORE_PID=$!
    sleep 1
    if [[ -z $(pgrep roscore) ]]; then
	echo "Error launching roscore. Make sure no other ros processes are running and try again."
	exit 1
    fi
fi
# Have to kill BG process if user exits
trap 'kill_processes $ROSCORE_PID' SIGINT
sleep 5

# If maze file argument is provided, pass it to the backend via ros param
# (code backend defaults to m1.maze so we don't worry about that case)
if [[ -n $maze_file ]]; then
    rosparam set /maze_file "$maze_file"
fi

# Node that displays the maze and runs the turtle
rosrun ece642rtle ece642rtle_node&
TURTLE_PID=$!
sleep 1
if [[ -z $(pgrep ece642rtle_node) ]]; then
    echo "Error launching ece642rtle_node. Have you run catkin_make?"
    kill_processes $ROSCORE_PID
    exit 1
fi
# Have to kill BG processes if user exits
trap 'kill_processes $TURTLE_PID $ROSCORE_PID' SIGINT
sleep 9

# Student node
rosrun ece642rtle ece642rtle_student&
STUDENT_PID=$!
# Have to kill BG processes if user exits
trap 'kill_processes $STUDENT_PID $TURTLE_PID $ROSCORE_PID' SIGINT

# Spin
while [ 1 -eq 1 ]; do
    sleep 30
done

