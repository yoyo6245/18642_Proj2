/*
 * Code by Milda Zizyte
 *
 * An example of a simple runtime monitor.
 * Handles message interrupts by printing the message contents
 * to the terminal.
 */

#include "monitor_interface.h"

/*
 * This interrupt occurs when a call to moveTurtle returns.
 * It can be used to keep track of what messages are being sent in a
 * single call to moveTurtle.
 */
void tickInterrupt(ros::Time t) {
  ROS_INFO("[[%ld ns]] One call to moveTurtle made.", t.toNSec());
}

void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  std::string o_str;
  switch(o) {
  case NORTH:
    o_str = "NORTH";
    break;
  case WEST:
    o_str = "WEST";
    break;
  case SOUTH:
    o_str = "SOUTH";
    break;
  case EAST:
    o_str = "EAST";
    break;
  default:
    o_str = "ERROR";
    break;
  }
  ROS_INFO("[[%ld ns]] 'Pose' was sent. Data: x = %d, y=%d, o=%s", t.toNSec(), x, y, o_str.c_str());
}

void visitInterrupt(ros::Time t, int visits) {
  ROS_INFO("[[%ld ns]] 'Visits' was sent. Data: %d", t.toNSec(), visits);
}

// t is the time of the bump request, not answer
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
  ROS_INFO("[[%ld ns]] 'Bump request' was sent. Data: x1 = %d, y1 = %d, x2 = %d, y2 = %d, resp = %s", t.toNSec(), x1, y1, x2, y2, bumped ? "true" : "false");
}

// t is the time of the at_end request, not answer
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
  ROS_INFO("[[%ld ns]] 'At End Request' was sent. Data: x = %d, y = %d, resp = %s", t.toNSec(), x, y, atEnd ? "true" : "false");
}
