#!/bin/bash

# Will kill all processes that are passed as arguments
# Used with trap to guarantee that CTRL+C on this script kills all BG processes
# Parses logs for violations while killing
kill_processes() {
    touch VIOLATIONS.txt
    rm VIOLATIONS.txt
    total_viol=0
    while [[ $# -gt 0 ]]; do
	m=$1
	shift
	if [[ -n $(pgrep -x ${m:0:15}) ]]; then
	    echo "Killing monitor $m"
	    kill $(pgrep -x ${m:0:15})
	    sleep 1
	    # Process any non-empty monitor file
	    if [ -s "$m.output.tmp" ]; then
		echo "" >> VIOLATIONS.txt
		echo "Monitor $m Violations:" >> VIOLATIONS.txt
		echo "" >> VIOLATIONS.txt
		grep -C 5 "[ WARN]" $m.output.tmp >> VIOLATIONS.txt
		m_viol=`grep "[ WARN]" $m.output.tmp | wc -l`
		total_viol=$(( total_viol + m_viol))
		rm $m.output.tmp
		echo "" >> VIOLATIONS.txt
	    fi
	fi
    done
    echo "TOTAL VIOLATIONS: $total_viol"
    echo "Any violations logged in VIOLATIONS.txt"
    echo "killed all processes, exiting"
    exit 0
}

if [[ $# -eq 0 ]]; then
    echo "Must provide at least one argument"
    echo "Usage: ./run_642_monitors.sh [-h|--help] MONITOR_1_NAME ..."
    exit 1
fi

if [[ "$1" == "-h" || "$1" == "--help" ]]; then
    echo "Usage: ./run_642_monitors.sh [-h|--help] MONITOR_1_NAME ..."	
    exit 2
fi

# Locate ece642rtle on system
turtledir=`rospack find ece642rtle`
if [ -z "$turtledir" ]; then
    echo "Cannot locate package ece642rtle."
    echo "Did you run catkin_make in the directory before proceeding?"
    exit 1
fi


# So ROS can find everything (catkin_ws is two directories above package directory)
source $turtledir/../../devel/setup.bash

MONITORS=""

for mon in "$@"
do
    if [[ "${mon:0:6}" != "ece642" ]]; then
	echo "Monitor $m does not begin with ece642! Exiting"
	kill_processes $MONITORS
	echo "Monitor $m does not begin with ece642! Exiting"
    fi
    echo "Starting monitor $mon"
    stdbuf -oL rosrun ece642rtle $mon | tee $mon.output.tmp&
    sleep 1
    # Need -f for full name because process name might be long
    if [[ -z $(pgrep ${mon:0:15}) ]]; then
	echo "Error launching $mon. Have you run catkin_make?"
	rm $mon.output.tmp
	kill_processes $MONITORS
	echo "Error launching $mon. Have you run catkin_make?"
	exit 1
    fi
    # Have to kill BG processes if user exits
    MONITORS="$MONITORS $mon"
    trap 'kill_processes $MONITORS' SIGINT
done

# Have to kill BG processes if user exits
trap 'kill_processes $MONITORS' SIGINT

# Spin
while [ 1 -eq 1 ]; do
    sleep 30
done

