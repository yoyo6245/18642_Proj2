#include "monitor_interface.h"

static Orientation last_dir;
static bool turned = false;
static const bool suppress_double_visits = true;

/*
 * Whenever the turtle turns, compare the current direction to the previous
 * direction and throw an invariant violation if the directions differ by 
 * more than 90 degree
 */
void poseInterrupt(ros::Time t, int x, int y, Orientation o){
  static std::string o_str;
  static std::string last_dir_str;
  switch(o){
    case WEST:
      o_str = "WEST";
      break;
    case NORTH:
      o_str = "NORTH";
      break;
    case EAST:
      o_str = "EAST";
      break;
    case SOUTH:
      o_str = "SOUTH";
      break;
    default:
      o_str = "out of range";
      break;
  }
  if(!suppress_double_visits || !turned || o != last_dir){
    ROS_INFO("[[%ld ns]] 'Turn' was sent. Data: orientation = %s", t.toNSec(), o_str );
  }
  // Check if the turtle has turned before and the degree does not exceed 90
  if(turned && ((o == WEST && last_dir == EAST) || (o == NORTH && last_dir == SOUTH) || (o == EAST && last_dir == WEST) || (o == SOUTH && last_dir == NORTH))){
    ROS_WARN("VIOLATION: Difference between last orientation and current orientation is more than 90 degree! (From %s to %s)", o_str, last_dir_str);
  }
  last_dir = o;
  last_dir_str = o_str;
  if(!turned){
    turned = true;
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

