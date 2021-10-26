/*
 * Originally by Philip Koopman (koopman@cmu.edu)
 * and Milda Zizyte (milda@cmu.edu)
 *
 * STUDENT NAME:YUAN-YU LEE
 * ANDREW ID:yuanyul
 * LAST UPDATE:2021/09/24
 *
 * This file is an algorithm to solve the ece642rtle maze
 * using the left-hand rule. The code is intentionaly left obfuscated.
 *
 */

#include "student.h"
#include "yuanyul_type.h"
#include <iostream>
#include <stack>
using namespace std;

static int time_cnt = 0;
static int turtle_map[MAP_SIZE][MAP_SIZE];
static Position relative_pos(INI_POS, INI_POS);
static STATE state = Detect;
static int relative_orientation = Left;
static stack<Position> track;
static int turn_cnt;

/**
  *@brief Let the turtle turn right based on its orientation
  */
void turn_right(){
  relative_orientation = (relative_orientation + 1) % DIRECTIONS;
  turn_cnt++;
  return ;
}

/**
  *@brief Let the turtle turn left based on its orientation
  */
void turn_left(){
  relative_orientation = (relative_orientation + DIRECTIONS - 1) % DIRECTIONS;
  turn_cnt++;
  return ;
}

/**
  *@brief get the value of a location in the 2D array
  *@param x: value of x-axis of the turtle
  *@param y: value of y-axis of the turtle
  */
int get_turtle_visit(int x, int y){
  return turtle_map[x][y];
}

/**
  *@brief get the value of current location in the 2D array
  */
int get_now_visit(){
  return get_turtle_visit(relative_pos.x, relative_pos.y);
}

/**
  *@brief set the specific value of a location in the 2D array
  *@param x: value of x-axis of the turtle map
  *@param y: value of y-axis of the turtle map
  */
void set_turtle_visit(int x, int y, int value){
  turtle_map[x][y] = value;
}

/**
  *@brief update the relative position of thr turtle
  */
void new_update_position(){
  switch(relative_orientation){
    case(Left):
      relative_pos.x -= 1;
      break;
    case(Up):
      relative_pos.y -= 1;
      break;
    case(Right):
      relative_pos.x += 1;
      break;
    case(Down):
      relative_pos.y += 1;
      break;
    default:
      ROS_ERROR("relative_orientation out of range!");
      break;
  }
  int val_visit = get_turtle_visit(relative_pos.x, relative_pos.y);
  set_turtle_visit(relative_pos.x, relative_pos.y, val_visit + 1);
  turn_cnt = 0;
}

/**
  *@brief return the position the turtle facing
  */
Position get_facing_pos(){
  Position facing_pos = relative_pos;
  switch(relative_orientation){
    case(Left):
      facing_pos.x -= 1;
      break;
    case(Up):
      facing_pos.y -= 1;
      break;
    case(Right):
      facing_pos.x += 1;
      break;
    case(Down):
      facing_pos.y += 1;
      break;
    default:
      ROS_ERROR("facing position out of range!");
      break;
  }
  return facing_pos;
}
 

/**
  *@brief check the cell that turtle facing is visited or not
  */
bool get_facing_visited(){
  Position facing_pos = get_facing_pos();
  return  get_turtle_visit(facing_pos.x, facing_pos.y) > 0;
}

/**
  *@brief check the cell that turtle facing is the cell it came from
  */
bool check_came_from(){
  Position facing_pos = get_facing_pos();
  return (facing_pos.x == track.top().x && facing_pos.y == track.top().y);
}

/**
  *@brief change the state and return value for different conditions
  *
  */
turtleMove handleBump(){
  state = Detect;
  turn_left();
  return turtleMove::LEFT;
}

turtleMove handleInitial(){
  state = Detect;
  turn_right();
  return turtleMove::RIGHT;
}

turtleMove handleMove(){
  state = Initial;
  track.push(relative_pos);
  ROS_INFO("push: %d, %d", relative_pos.x, relative_pos.y);
  new_update_position();
  return turtleMove::MOVE;
}

turtleMove handleDetect(bool bumped){
  if(turn_cnt > 3 && check_came_from()){
    state = Go_back;
  }
  else if(bumped || get_facing_visited()){
    state = Bump;
  }
  else{
    state = Wont_bump;
  }
  return turtleMove::NO_MOVE;
}

turtleMove handleGoback(){
  state = Initial;
  track.pop();
  ROS_INFO("Go back!");
  new_update_position();
  return turtleMove::MOVE;
}

/**
  *@brief let the time counter run
  *
  */
bool tiktok(){
  if(time_cnt == 0){
    time_cnt = TIMEOUT;
  }
  else{
    time_cnt -= 1;
  }
  return time_cnt == TIMEOUT;
}

// Ignore this line until project 5
turtleMove studentTurtleStep(bool bumped, bool atend) {
  turtleMove final_move = turtleMove::NO_MOVE;
  if(atend){
    return final_move; 
  }

  if(time_cnt == 0){
    //if(!track.empty())  ROS_INFO("track.top: (%d, %d)", track.top().x, track.top().y);
    switch(state){
      case(Bump):
        ROS_INFO("state = Bump");
        final_move = handleBump();
        break;
      case(Initial):
        ROS_INFO("state = Initial");
        final_move = handleInitial();
        break;
      case(Wont_bump):
        ROS_INFO("state = Wont Bump");
        final_move = handleMove();
        break;
      case(Detect):
        ROS_INFO("state = Detect");
        final_move = handleDetect(bumped);
        break;
      case(Go_back):
        ROS_INFO("state = Go back");
        final_move = handleGoback();
        break;
      default:
        ROS_ERROR("state out of range.");
        break;
    }
  }
  if(final_move != turtleMove::NO_MOVE){
    //ROS_INFO("Orientation=%d  STATE=%d rela_x: %d, rela_y: %d", relative_orientation, state, relative_pos.x, relative_pos.y);
  }
  
  bool reset_time_cnt = tiktok();
  if(!reset_time_cnt){
    final_move = turtleMove::NO_MOVE;
  }

  return final_move; 
}

// OK TO MODIFY BELOW THIS LINE

// this procedure takes the current turtle position and orientation and returns
// true=submit changes, false=do not submit changes
// Ground rule -- you are only allowed to call the helper functions "bumped(..)"
// and "atend(..)",
// and NO other turtle methods or maze methods (no peeking at the maze!)

