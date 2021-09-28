/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:Yuan-Yu Lee
 * ANDREW ID:yuanyul
 * LAST UPDATE:9/26
 *
 * This file keeps track of where the turtle is in the maze
 * and updates the location when the turtle is moved. It shall not
 * contain the maze solving logic/algorithm.
 *
 * This file is used along with student_turtle.cpp. student_turtle.cpp shall
 * contain the maze solving logic/algorithm and shall not make use of the
 * absolute coordinates or orientation of the turtle.
 *
 * This file shall call studentTurtleStep(..) in student_turtle.cpp to determine
 * the next move the turtle will make, and shall use translatePos(..) and
 * translateOrnt(..) to translate this move into absolute coordinates
 * to display the turtle.
 *
 */

#include "student.h"
#include "type.h"

static Position pos_start, pos_next;
static Orientation static_or;

/*
 * This procedure takes the current turtle position and orientation and returns true=accept changes, false=do not accept changes
 * Ground rule -- you are only allowed to call the three helper functions defined in student.h, and NO other turtle methods or maze methods (no peeking at the maze!)
 * This file interfaces with functions in student_turtle.cpp
 */

/**
  *@brief set next position based on the orientation
  *@param pos_: current position information
  *@param now_orientation: current orientation
  *@param pos_start: the position of starting point
  *@param pos_next: the position of next point
  */
void next_pos(QPointF &pos_, int now_orientation){
  pos_start.x = pos_.x();
  pos_start.y = pos_.y();
  pos_next.x = pos_.x();
  pos_next.y = pos_.y();
  static_or = static_cast<Orientation>(now_orientation);
  switch(static_or){
    case Left:
      pos_next.y += 1;
      break;
    case Up:
      pos_next.x += 1;
      break;
    case Right:
      pos_start.x += 1;
      pos_next.x += 1;
      pos_next.y += 1;
      break;
    case Down:
      pos_start.y += 1;
      pos_next.x += 1;
      pos_next.y += 1;
      break;
    default:
      ROS_ERROR("now_orientation is out of range.");
      break;
  }
  //ROS_INFO("start: %d, %d  next: %d, %d", pos_start.x, pos_start.y, pos_next.x, pos_next.y);
}


/*
 *@brief Check if the turtle bumped and change current position and 
 *orientation of the turtle
 *@param pos_start: the position of starting point 
 *@param pos_next: the position of next point
 *@param now_orientation the orientation of the turtle
 */
bool bump_sensor(QPointF &pos_, int now_orientation){
  next_pos(pos_, now_orientation);
  return bumped(pos_start.x, pos_start.y, pos_next.x, pos_next.y);
}


bool moveTurtle(QPointF& pos_, int& now_orientation)
{
  bool bumped = bump_sensor(pos_, now_orientation); // Replace with your own procedure
  bool at_end = atend(pos_.x(), pos_.y());
  bool move_turtle = false;

  turtleMove nextMove = studentTurtleStep(bumped, at_end); // define your own turtleMove enum or structure
  if(nextMove != turtleMove::NO_MOVE){
    move_turtle = true;
    pos_ = translatePos(pos_, nextMove);
    now_orientation = translateOrnt(now_orientation, nextMove);
    ROS_INFO("abs x: %f, y: %f, orientation: %d", pos_.x(), pos_.y(), now_orientation);
  }
  // REPLACE THE FOLLOWING LINE IN PROJECT 5
  return move_turtle;
  //return studentMoveTurtle(pos_, now_orientation);
}

/**
  *@brief update the position of thr turtle
  *@param pos_: position information of the turtle
  */

QPointF update_pos(QPointF &pos_){
  switch(static_or){
    case(Left):
      ROS_INFO("MOVE LEFT!");
      pos_.setX(pos_.x() - 1);
      break;
    case(Up):
      ROS_INFO("MOVE UP!");
      pos_.setY(pos_.y() - 1);
      break;
    case(Right):
      ROS_INFO("MOVE RIGHT!");
      pos_.setX(pos_.x() + 1);
      break;
    case(Down):
      ROS_INFO("MOVE DOWN!");
      pos_.setY(pos_.y() + 1);
      break;
    default:
      ROS_ERROR("orientation is out of range.");
      break;
  }
  return pos_;
}

/**
  *@brief let the turtle turn clockwise
  *@param now_orientation: the orientation of the turtle
  */
int absolute_turn_right(int now_orientation){
  now_orientation = (now_orientation + 1) % DIRECTIONS;
  Orientation ori_term = static_cast<Orientation>(now_orientation);
  return ori_term;
}

/**
  *@brief let the turtle turn counterclockwise
  *@param now_orientation: the orientation of the turtle
  */
int absolute_turn_left(int now_orientation){
  now_orientation = (now_orientation + DIRECTIONS - 1) % DIRECTIONS;
  Orientation ori_term = static_cast<Orientation>(now_orientation);
  return ori_term;
}


/*
 * Takes a position and a turtleMove and returns a new position
 * based on the move
 */
QPointF translatePos(QPointF pos_, turtleMove nextMove) {
  switch(nextMove){
      ROS_INFO("nextMove == NO_MOVE");
    case turtleMove::NO_MOVE:
      break;
    case turtleMove::MOVE:
      ROS_INFO("nextMove == MOVE");
      pos_ = update_pos(pos_);
      displayVisits(get_now_visit());
      break;
    case turtleMove::LEFT:
      ROS_INFO("nextMove == LEFT");
      break;
    case turtleMove::RIGHT:
      ROS_INFO("nextMove == RIGHT");
      break;
    default:
      break;
  }

  return pos_;
}

/*
 * Takes an orientation and a turtleMove and returns a new orienation
 * based on the move
 */
int translateOrnt(int now_orientation, turtleMove nextMove) {
  switch(nextMove){
    case turtleMove::NO_MOVE:
      break;
    case turtleMove::MOVE:
      break;
    case turtleMove::LEFT:
      now_orientation = absolute_turn_left(now_orientation);
      break;
    case turtleMove::RIGHT:
      now_orientation = absolute_turn_right(now_orientation);
      break;
    default:
      break;
  }
  return now_orientation;
}
