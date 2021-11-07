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

static bool mock_error = false;

void setMockerror(bool new_mock_error){
  mock_error = new_mock_error;
}

bool getMockerror(){
  return mock_error;
}

/* Dummy ROS_ERROR */
void ROS_ERROR(std::string e) {
  mock_error = true;
  std::cout << e << std::endl;
}

void ROS_INFO(std::string i) {
  std::cout << i << std::endl;
}
