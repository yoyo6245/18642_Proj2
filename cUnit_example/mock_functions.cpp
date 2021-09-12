/*
 * 18-642 Unit Testing Example
 * Example code by Milda Zizyte
 */

#include "student_mock.h"
#include <iostream>

static orientation mock_orientation;
static bool mock_bump;
static bool mock_error = false;

/* Functions called by dummy_turtle */
void setOrientation(orientation ornt) {
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
