/*
 * 18-642 Unit Testing Example
 * Example code by Milda Zizyte
 */

#include <iostream>

enum move_state {MOVE_FORWARD, MOVE_BACK};
enum orientation {UP, DOWN};

move_state moveTurtle(move_state curr_state, bool atEnd);

// Mock student_maze functions
void setOrientation(orientation);
bool will_bump();

// Functions called by main testing program to get or set values
orientation test_orientation_result();
void mock_set_bump(bool bump);

void ROS_ERROR(std::string e);
