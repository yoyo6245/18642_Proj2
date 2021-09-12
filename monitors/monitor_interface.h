/*
 * Code by Milda Zizyte
 *
 * Interface of ece642rtle system ROS message interrupt handling
 * intended to be used for runtime monitoring
 */

#include <boost/bind.hpp>
#include <ros/ros.h>
#include <ece642rtle/PoseOrntBundle.h>
#include <ece642rtle/timeInt8.h>
#include <ece642rtle/RTIbump.h>
#include <ece642rtle/RTIatend.h>
#include <ece642rtle/bumpEcho.h>
#include <ece642rtle/aendEcho.h>

// (x,y) coordinates of a turtle pose
typedef struct{int x; int y;} Pose;
// endpoints of a potential wall segment
typedef struct {int x1; int y1; int x2; int y2;} Endpoints;
/*
 * Relationship between pose and endpoints: 
 *   (x,y)+-----+(x+1,y)
 *        |     |
 *        |(x,y)|
 *        |     |
 * (x,y+1)+-----+(x+1,y+1)
 */

enum Orientation {WEST, NORTH, EAST, SOUTH};

// Occurs every time moveTurtle is called
void tickInterrupt(ros::Time t);
// Occurs every time turtle pose and orientation is updated
void poseInterrupt(ros::Time t, int x, int y, Orientation o);
// Occurs every time the visit count at the current location is updated
void visitInterrupt(ros::Time t, int visits);
// Occurs every time a call to bumped(x1,y1,x2,y2) returns
// (t is the time of the server request, not answer)
void bumpInterrupt(ros::Time t, int x1, int y1, int x2, int y2, bool bumped);
// Occurs every time a call to atend(x,y) returns
// (t is the time of the server request, not answer)
void atEndInterrupt(ros::Time t, int x, int y, bool atEnd);
