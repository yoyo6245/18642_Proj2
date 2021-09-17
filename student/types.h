typedef unsigned int TIME_CNT;
typedef struct Position {
  int x;
  int y;
}POSITION;
typedef enum orientation {
  Right = 0,
  Left = 1,
  Forward = 2,
  Back = 3
} ORIENTATION;
typedef enum state {
  Bump, Initial, Wont_bump 
} STATE;
typedef unsigned int TIME_CNT;
const unsigned int TIMEOUT = 30;
