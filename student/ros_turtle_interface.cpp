/*
 * 18-642 ros_turtle_interface.cpp
 *
 * This file describes the interface between the ece642rtle_student ROS node
 * and the student code. It defines the communication mechanism between
 * ece642rtle_node and ece642rtle_student and calls the moveTurtle function
 * in student_maze.cpp to invoke student code.
 *
 * At a high level, every time ece642rtle_student sends a pose update or a blank
 * message to ece642rtle_node, ece642rtle_node updates the turtle position/orientation
 * (if any) and sends back pose and orientation data. The receipt of this message
 * by ece642rtle_student prompts a call to moveTurtle.
 * 
 * There are also calls to bumped(), atend(), and displayVisits().
 *
 * STUDENTS SHOULD NOT MODIFY THIS FILE.
 *
 * Code by Milda Zizyte
 */

#include "student.h"
QPointF TRI_pose_;
bool TRI_at_end_;

ros::Publisher TRI_req_;
ros::Publisher TRI_pub_;
ros::Publisher TRI_vist_pub_;
ros::Subscriber RTI_sub_;
ros::ServiceClient RTIbumpClient;
ros::ServiceClient RTIatendClient;
ros::Publisher RTI_bump_echo;
ros::Publisher RTI_aend_echo;

/*
 * A call to bumped(..) results in a call to the ROS service
 * RTIbump, which returns true if the maze contains a wall
 * between endpoints (x1,y1) and (x2,y2) and false otherwise.
 * We also send out the message in an "echo" so that runtime 
 * monitors can see it.
 */
bool bumped(int x1, int y1, int x2, int y2) {
  ece642rtle::RTIbump bump_srv;
  bump_srv.request.x1 = (signed char) x1;
  bump_srv.request.x2 = (signed char) x2;
  bump_srv.request.y1 = (signed char) y1;
  bump_srv.request.y2 = (signed char) y2;
  ros::Time req_time = ros::Time::now();
  // Make the ROS service call and check for success
  if (RTIbumpClient.call(bump_srv)) {
    bool resp = bump_srv.response.bump;
    // Echo arguments and response
    ece642rtle::bumpEcho bump_echo;
    bump_echo.t = req_time;
    bump_echo.x1 = x1;
    bump_echo.y1 = y1;
    bump_echo.x2 = x2;
    bump_echo.y2 = y2;
    bump_echo.resp = resp;
    RTI_bump_echo.publish(bump_echo);
    return resp;
  } else {
    ROS_ERROR("Failed to call service bump");
    return true;
  }
}

/*
 * A call to atend(..) results in a call to the ROS service
 * RTIatend, which returns true if the square (x,y) is the end 
 * cell of the maze and false otherwise. 
 * We also send out the message in an "echo" so that runtime
 * monitors can see it.
 */
bool atend(int x, int y) {
  ece642rtle::RTIatend atend_srv;
  atend_srv.request.x = (signed char) x;
  atend_srv.request.y = (signed char) y;
  ros::Time req_time = ros::Time::now();
  // Make the ROS service call and check for success
  if (RTIatendClient.call(atend_srv)) {
    bool resp = atend_srv.response.atend;
    // Echo arguments and response
    ece642rtle::aendEcho aend_echo;
    aend_echo.t = req_time;
    aend_echo.x = x;
    aend_echo.y = y;
    aend_echo.resp = resp;
    RTI_aend_echo.publish(aend_echo);
    return resp;
  } else {
    ROS_ERROR("Failed to call service at end");
    return false;
  }
}

/*
 * A call to displayVisits(..) will update the visit count of
 * the cell that the turtle is at when moveTurtle RETURNS.
 */
void displayVisits(int visits) {
  ece642rtle::timeInt8 v;
  v.data = (int8_t) visits;
  v.t = ros::Time::now();
  TRI_vist_pub_.publish(v);
}

/*
 * Receive pose and orientation data from ece642rtle_node. This means
 * we are ready for the next turtle move, so we call moveTurtle
 * and then send a message to ece642rtle_node to let it know that something
 * was updated.
 * moveTurtle returns "false" if no changes should be sent to ece642rtle_node.
 * In this case, we send an empty message to ece642rtle_node so that it knows
 * to try to send another pose and orientation message.
 */
void RTIcallback(const ece642rtle::PoseOrntBundleConstPtr& RTI_pob) {
  QPointF RTI_pos(RTI_pob->x, RTI_pob->y);
  int RTI_o = RTI_pob->o;
  // These are passed by reference
  bool move = moveTurtle(RTI_pos, RTI_o);
  if (move) {
    ece642rtle::PoseOrntBundle TRI_pob;
    TRI_pob.x = (int8_t) RTI_pos.x();
    TRI_pob.y = (int8_t) RTI_pos.y();
    TRI_pob.o = (int8_t) RTI_o;
    TRI_pob.t = ros::Time::now();
    TRI_pub_.publish(TRI_pob);
  } else {
    TRI_req_.publish(std_msgs::Empty());
  }
}

/*
 * Register publishers, subscribers, services and send an empty message
 * to kick things off.
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_turtle_interface");
    ros::NodeHandle nh;
    TRI_req_ = nh.advertise<std_msgs::Empty>("turtle1/TRI_req", 1);
    TRI_pub_ = nh.advertise<ece642rtle::PoseOrntBundle>("turtle1/TRI_pob", 1);
    TRI_vist_pub_ = nh.advertise<ece642rtle::timeInt8>("turtle1/TRI_vist", 1);
    RTI_sub_ = nh.subscribe("turtle1/RTI_pob", 1, RTIcallback);
    RTIbumpClient = nh.serviceClient<ece642rtle::RTIbump>("turtle1/RTI_bump");
    RTIatendClient = nh.serviceClient<ece642rtle::RTIatend>("turtle1/RTI_at_end");
    RTI_bump_echo = nh.advertise<ece642rtle::bumpEcho>("turtle1/RTI_bump_echo",1);
    RTI_aend_echo = nh.advertise<ece642rtle::aendEcho>("turtle1/RTI_aend_echo",1);
    TRI_at_end_ = false;

    ros::Rate loop_rate(10);
    // Don't start sending until we're online
    while(TRI_pub_.getNumSubscribers() < 1) {
      loop_rate.sleep();
    }
    loop_rate.sleep();
    loop_rate.sleep();
    
    TRI_req_.publish(std_msgs::Empty());
    ros::spinOnce();
    
    ros::spin();
}
