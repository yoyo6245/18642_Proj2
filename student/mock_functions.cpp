/*
 * Originally by Phillip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:YUAN-YU LEE
 * ANDREW ID:yuanyul
 * LAST UPDATE:2021/11/05
 *
 */

#include "student_mock.h"
#include <iostream>

static Orientation mock_orientation;
static bool mock_bump;
static bool mock_error = false;

void setMockerror(bool new_mock_error){
  mock_error = new_mock_error;
}

bool getMockerror(){
  return mock_error;
}

/* Functions called by dummy_turtle */
void setOrientation(Orientation ornt) {
  mock_orientation = ornt;
}

bool will_bump() {
  return mock_bump;
}

/* Functions used to instrument CUnit tests */
orientation test_orientation_result() {
  return mock_orientation;
}

void mock_set_bump(bool bump) {
  mock_bump = bump;
}

/* Dummy ROS_ERROR */
void ROS_ERROR(std::string e) {
  mock_error = true;
  std::cout << e << std::endl;
}
