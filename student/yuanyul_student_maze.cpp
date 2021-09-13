/* 
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:
 * ANDREW ID:
 * LAST UPDATE:
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

/*
 * This procedure takes the current turtle position and orientation and returns true=accept changes, false=do not accept changes
 * Ground rule -- you are only allowed to call the three helper functions defined in student.h, and NO other turtle methods or maze methods (no peeking at the maze!)
 * This file interfaces with functions in student_turtle.cpp
 */
bool moveTurtle(QPointF& pos_, int& nw_or)
{
  bool bumped = true; // Replace with your own procedure
  turtleMove nextMove = studentTurtleStep(bumped); // define your own turtleMove enum or structure
  pos_ = translatePos(pos_, nextMove);
  nw_or = translateOrnt(nw_or, nextMove);

  // REPLACE THE FOLLOWING LINE IN PROJECT 5
  return studentMoveTurtle(pos_, nw_or);
}

/*
 * Takes a position and a turtleMove and returns a new position
 * based on the move
 */
QPointF translatePos(QPointF pos_, turtleMove nextMove) {
  return pos_;
}

/*
 * Takes an orientation and a turtleMove and returns a new orienation
 * based on the move
 */
int translateOrnt(int orientation, turtleMove nextMove) {
  return orientation;
}
