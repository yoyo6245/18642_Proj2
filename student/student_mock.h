/*   
 * Originally by Philip Koopman (koopman@cmu.edu)     
 * and Milda Zizyte (milda@cmu.edu)     
 *
 * STUDENT NAME:YUAN-YU LEE         
 * ANDREW ID:yuanyul                                            
 * LAST UPDATE:2021/11/05          
 *                                      
 */

#include "yuanyul_type.h"
#include <iostream>

enum turtleMove {NO_MOVE = 0, MOVE, LEFT, RIGHT};

// Mock student_maze functions
Position get_facing_pos();
Position get_position();
void new_update_position();
void turn_right();
void turn_left();
void set_came_from();
void setTime_cnt(int new_time_cnt);
void setTurn_cnt(int new_turn_cnt);
void setOrientation(Orientation new_Orientation);
Orientation getOrientation();
void setState(STATE new_state);
STATE getState();
bool will_bump();
turtleMove studentTurtleStep(bool bumped, bool atend);

// Functions called by main testing program to get or set values
Orientation test_orientation_result();
void mock_set_bump(bool bump);

void ROS_ERROR(std::string e);
void ROS_INFO(std::string i);
void setMockerror(bool new_mock_error);
bool getMockerror();
