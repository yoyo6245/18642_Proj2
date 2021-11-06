/*   
 * Originally by Philip Koopman (koopman@cmu.edu)     
 * and Milda Zizyte (milda@cmu.edu)     
 *
 * STUDENT NAME:YUAN-YU LEE         
 * ANDREW ID:yuanyul                                            
 * LAST UPDATE:2021/11/05          
 *                                      
 */


#include <iostream>

enum turtleMove {NO_MOVE = 0, MOVE, LEFT, RIGHT};
typedef enum orientation {
  Left, Up, Right, Down
}Orientation;
typedef enum state{
  Bump, Initial, Wont_bump, Detect, Go_back
}STATE;
typedef struct Position {
  int x;
	int y;
	Position(){x = 0; y = 0;}
	Position(int x_in, int y_in){x = x_in; y = y_in;}
}Position;

//move_state moveTurtle(move_state curr_state, bool atEnd);

// Mock student_maze functions
void setOrientation(Orientation new_Orientation);
Orientation getOrientation();
void setState(STATE new_state);
State getState();
bool will_bump();

// Functions called by main testing program to get or set values
Orientation test_orientation_result();
void mock_set_bump(bool bump);

void ROS_ERROR(std::string e);
void setMockerror(bool new_mock_error);
bool getMockerror();
