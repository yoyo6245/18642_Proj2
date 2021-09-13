/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:YUAN-YU LEE
 * ANDREW ID:yuanyul
 * LAST UPDATE:2021/09/12
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) { return MOVE; }

// OK TO MODIFY BELOW THIS LINE

#define TIMEOUT                                                                \
  40 // bigger number slows down simulation so you can see what's happening
float w, cs;              // w: time counter   cs: state
float fx1, fy1, fx2, fy2; // 1: start  2: end
float z, aend, mod, bp, q;

enum direction {
	Right = 0,
	Left = 1,
	Forward = 2,
	Back = 3
};

// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)"
// and "atend(..)",
// and NO other turtle methods or maze methods (no peeking at the maze!)
bool studentMoveTurtle(QPointF &pos_, int &nw_or) {
  // nw_or = orientation 0:look right 1:look left 2:move forward

  // ROS_INFO("Turtle update Called  w=%f", w);
  mod = true;
  if (w == Right) {
    fx1 = pos_.x();
    fy1 = pos_.y();
    fx2 = pos_.x();
    fy2 = pos_.y();
    if (nw_or < 2)
      if (nw_or == Right) // nw_or == 0 look right
        fy2 += 1;
      else // nw_or == 1 look left
        fx2 += 1;
    else { // nw_or >= 2
      fx2 += 1;
      fy2 += 1;
      if (nw_or == Forward) // nw_or == 2 move forward
        fx1 += 1;
      else
        fy1 += 1;
    }
    bp = bumped(fx1, fy1, fx2, fy2);  // try sensor
    aend = atend(pos_.x(), pos_.y()); // check new pos
    if (nw_or == Right) 			// last: look right
      if (cs == 2) {
        nw_or = Back;
        cs = 1;
      } else if (bp) {			// look left
        nw_or = Left;
        cs = 0;
      } else
        cs = 2;
    else if (nw_or == Left)	// last: look left
      if (cs == 2) {
        nw_or = Right;
        cs = 1;
      } else if (bp) {			// if can bump -> move forward
        nw_or = Forward;
        cs = 0;
      } else
        cs = 2;
    else if (nw_or == Forward)	// last: move forward
      if (cs == 2) {
        nw_or = Left;
        cs = 1;
      } else if (bp) {			// if can bump -> ?
        nw_or = Back;
        cs = 0;
      } else
        cs = 2;
    else if (nw_or == Back)
      if (cs == 2) {
        nw_or = Forward;
        cs = 1;
      } else if (bp) {
        nw_or = Right;
        cs = 0;
      } else
        cs = 2;
    ROS_INFO("Orientation=%f  STATE=%f", nw_or, cs);
    z = cs == 2;
    mod = true;
    if (z == true && aend == false) {
      if (nw_or == Left)
        pos_.setY(pos_.y() - 1);
      if (nw_or == Forward)
        pos_.setX(pos_.x() + 1);
      if (nw_or == Back)
        pos_.setY(pos_.y() + 1);
      if (nw_or == Right)
        pos_.setX(pos_.x() - 1);
      z = false;
      mod = true;
    }
  }
  if (aend)
    return false;
  if (w == 0)
    w = TIMEOUT;
  else
    w -= 1;
  if (w == TIMEOUT)
    return true;
  return false;
}
