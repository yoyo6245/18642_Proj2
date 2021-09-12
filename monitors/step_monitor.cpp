/*
 * Code by Milda Zizyte
 *
 * This monitor checks that the invariant "turtle shall not move more
 * than on square at a time" is not violated.
 * It keeps track of the previous position of the turtle and compares it
 * to the current position to check the invariant.
 */

#include "monitor_interface.h"

// Keeps track of the last pose received
// moved is true if at least one pose has been received, false otherwise
static Pose last_pose;
static bool moved = false;

// Flag that doesn't print pose updates if the turtle has moved 0 steps
static const bool suppress_double_visits = true;

/*
 * Returns absolute value of x
 * WARNING: unsafe for edge-case values
 */
inline int abs(int x) {
  return x < 0 ? -1*x : x;
}

/*
 * Whenever the turtle moves, compare the current location
 * to the previous location and throw an invariant violation
 * if the locations differ by more than 1 in Manhattan Distance.
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o) {
  // Print pose info
  // Last conditional makes sure that if suppress_double_visits is
  // true, that the same pose isn't printed twice
  if (!suppress_double_visits || !moved ||
      (last_pose.x != x || last_pose.y != y)) {
    ROS_INFO("[[%ld ns]] 'Pose' was sent. Data: x = %d, y=%d", t.toNSec(), x, y);
  }

  // Check that the turtle has moved before and that the Manhattan
  // distance between the positions does not exceed 1
  if (moved && (abs(last_pose.x - x) + abs(last_pose.y - y)) > 1) {
    ROS_WARN("VIOLATION: Difference between last coordinate (%d,%d) and current coordinate (%d,%d) is more than 1 square!", last_pose.x, last_pose.y, x, y);
  }

  // store last Pose in memory
  last_pose.x = x;
  last_pose.y = y;

  // Update this flag the first time the turtle moves
  if (!moved) {
    moved = true;
  }
}

/*
 * Empty interrupt handlers beyond this point
 */

void tickInterrupt(ros::Time t) {
}

void visitInterrupt(ros::Time t, int visits) {
}

void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped) {
}

void atEndInterrupt(ros::Time t, int x, int y, bool atEnd) {
}
