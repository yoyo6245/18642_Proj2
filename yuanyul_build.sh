#!/bin/bash
cd ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
catkin_make ece642rtle_student
source ~/catkin_ws/devel/setup.bash
catkin_make ece642rtle_node
source ~/catkin_ws/devel/setup.bash
catkin_make ece642rtle_student
cd ~/catkin_ws/src/ece642rtle/student
g++ -Dtesting -o yuanyul_build_run_test.sh student_test.cpp yuanyul_student_turtle.cpp mock_functions.cpp -lcunit
./yuanyul_build_run_test.sh
cd ~/catkin_ws/src/ece642rtle
