/*
 * 18-642 Unit Testing Example
 * Example code by Milda Zizyte
 */


#ifdef testing
#include "student_mock.h"
#endif
#ifndef testing
#include "student.h"
#include "ros/ros.h"
#endif

move_state moveTurtle(move_state curr_state, bool at_end) {
  switch (curr_state) {
  case MOVE_FORWARD: //S1
    // Outputs
    setOrientation(UP);
    if (at_end) { // T1
      curr_state = MOVE_FORWARD;
    } else if (will_bump()) { // T2
      curr_state = MOVE_BACK;
    } else { // Default: no transition
      curr_state = MOVE_FORWARD;
    }
    break;
  case MOVE_BACK: // S2
    // Outputs
    setOrientation(DOWN);
    if (at_end) { // T3
      curr_state = MOVE_BACK;
    } else if (will_bump()) { // T4
      curr_state = MOVE_FORWARD;
    } else { // Default: no transition
      curr_state = MOVE_BACK;
    }
    break;
  default:
    ROS_ERROR("Reached default state");
    break;
  }
  return curr_state;
}
