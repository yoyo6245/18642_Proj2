/**
  * @brief struct to store pair for coordinated.
  *
  */
typedef struct Position {
	uint8_t x;
	uint8_t y;
	Position(){x = 0; y = 0;}
	Position(uint8_t x_in, uint8_t y_in){x = x_in; y = y_in;}
}Position;


/**
  * @brief Enum for orientation of turtle
  *
  * Left: 0     Turtle faces leftwards
  * Up:   1     Turtle faces upwards
  * Right:2     Turtle faces rightwards
  * Down: 3     Turtle faces downwards
  *
  */
typedef enum orientation {
  Left, Up, Right, Down
}Orientation;


/**
  *@brief Enum for current state of turtle
  * Bump:       0   Turtle will bump if moving forward
  * Initial:    1   Turtle just moved foward
  * Wont_bymp:  2   Turtle wont bump if moving forward
  *
  */
typedef enum state{
  Bump, Initial, Wont_bump, Detect
}STATE;

int get_now_visit();

// bigger value slows down simulation
static const unsigned int TIMEOUT = 5; 
// number of directions
static const unsigned int DIRECTIONS = 4;
// initial position will be set on map[11][11]
static const unsigned int INI_POS = 11;
static const unsigned int MAP_SIZE = 23;
