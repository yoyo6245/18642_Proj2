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
#include "types.h"
  
// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped) { return MOVE; }

// OK TO MODIFY BELOW THIS LINE

#define TIMEOUT                                                                \
  40 // bigger number slows down simulation so you can see what's happening
//float time_cnt, state;              // w: time counter   cs: state
//float fx1, fy1, fx2, fy2; // 1: start  2: end
//float pass_flag, reach_goal, bump_sensor;

// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)"
// and "atend(..)",
// and NO other turtle methods or maze methods (no peeking at the maze!)
bool studentMoveTurtle(QPointF &pos_, int &now_orientation) {
  // nw_or = orientation 0:look right 1:look left 2:move forward

  // ROS_INFO("Turtle update Called  w=%f", w);
  //mod = true;
  static STATE state;
  static TIME_CNT time_cnt;
  static POSITION pos_start, pos_next;
  static bool pass_flag, reach_goal, bump_sensor;
  if (time_cnt == 0) {
    pos_start.x = pos_.x();
    pos_start.y = pos_.y();
    pos_next.x = pos_.x();
    pos_next.y = pos_.y();
    switch(now_orientation){
      case Right:
        pos_next.y += 1;
        break;
      case Left:
        pos_next.x += 1;
        break;
      case Forward:
        pos_start.x += 1;
        pos_next.x += 1;
        pos_next.y += 1;
        break;
      case Back:
        pos_start.y += 1;
        pos_next.x += 1;
        pos_next.y += 1;
        break;
      default:
        ROS_ERROR("now_orientation is out of range.");
        break;
    }
    // check if there is a wall between (fx1, fy1) and (fx2, fy2)
    bump_sensor = bumped(pos_start.x, pos_start.y, pos_next.x, pos_next.y);
    // check if (x, y) is the end cell of the maze
    reach_goal = atend(pos_.x(), pos_.y()); 
		// cs = state  0: initial 1: will bump 2: wont bump
    switch(now_orientation){
      case (Right):
        if (state == Wont_bump) {
          now_orientation = Back;
          state = Initial;
        } else if (bump_sensor) {			// look left
          now_orientation = Left;
          state = Bump;
        } else {
          state = Wont_bump;
        }
        break;
      case (Left):
        if (state == Wont_bump) {
          now_orientation = Right;
          state = Initial;
        } else if (bump_sensor) {			// if can bump -> move forward
          now_orientation = Forward;
          state = Bump;
        } else {
          state = Wont_bump;
        }
        break;
      case (Forward):
        if (state == Wont_bump) {
          now_orientation = Left;
          state = Initial;
        } else if (bump_sensor) {	
          now_orientation = Back;
          state = Bump;
        } else {
          state = Wont_bump;
        }
        break;
      case (Back):
        if (state == Wont_bump) {
          now_orientation = Forward;
          state = Initial;
        } else if (bump_sensor) {
          now_orientation = Right;
          state = Bump;
        } else {
          state = Wont_bump;
        }
        break;
      default:
        ROS_ERROR("now_orientation is out of range.");
        break;
    }

    ROS_INFO("Orientation=%d  STATE=%d", now_orientation, state);
    
		pass_flag = state == Wont_bump;
    //mod = true;
    if (pass_flag == true && reach_goal == false) {
      switch(now_orientation){
        case(Left): 
          pos_.setY(pos_.y() - 1);
          break;
        case(Forward):
          pos_.setX(pos_.x() + 1);
          break;
        case(Back):
          pos_.setY(pos_.y() + 1);
          break;
        case(Right):
          pos_.setX(pos_.x() - 1);
          break;
        default:
          ROS_ERROR("now_orientation is out of range.");
          break;
      }
      pass_flag = false;
      //mod = true;
    }
  }
  if (reach_goal) {
    return false;
  }
  if (time_cnt == 0) {
    time_cnt = TIMEOUT;
  }
  else {
    time_cnt -= 1;
  }
  if (time_cnt == TIMEOUT) {
    return true;
  }
  return false;
}
