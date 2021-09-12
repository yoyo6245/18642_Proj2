/*
 * Code by Milda Zizyte
 *
 * ROS message handling for the interface to ece642rtle system
 * ROS message interrupt handling intended to be used for runtime monitoring
 */

#include "monitor_interface.h"

ros::Subscriber RTI_pob_sub_;
ros::Subscriber TRI_pob_sub_;
ros::Subscriber TRI_ornt_sub_;
ros::Subscriber TRI_vist_sub_;
ros::Subscriber bump_echo_sub_;
ros::Subscriber aend_echo_sub_;

void RTIpobCallback(const ece642rtle::PoseOrntBundleConstPtr& b) {
  tickInterrupt(b->t);
}

void TRIpobCallback(const ece642rtle::PoseOrntBundleConstPtr& p) {
  Orientation o_;
  switch(p->o) {
  case 0:
    o_ = WEST;
    break;
  case 1:
    o_ = NORTH;
    break;
  case 2:
    o_ = EAST;
    break;
  case 3:
    o_ = SOUTH;
    break;
  default:
    ROS_ERROR("Bad orientation %d", p->o);
    break;
  }
  poseInterrupt(p->t, (int) p->x, (int) p->y, o_);
}

void TRIvistCallback(const ece642rtle::timeInt8ConstPtr& v) {
  if ((int)v->data > -1) {
    visitInterrupt(v->t, v->data);
  }
}

void bumpEchoCallback(const ece642rtle::bumpEchoConstPtr& e) {
  bumpInterrupt(e->t, e->x1, e->y1, e->x2, e->y2, e->resp);
}


void aendEchoCallback(const ece642rtle::aendEchoConstPtr& p) {
  atEndInterrupt(p->t, p->x, p->y, p->resp);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ece642rtle_monitor", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    RTI_pob_sub_ = nh.subscribe("turtle1/RTI_pob",1, RTIpobCallback);
    TRI_pob_sub_ = nh.subscribe("turtle1/TRI_pob",1, TRIpobCallback);
    TRI_vist_sub_ = nh.subscribe("turtle1/TRI_vist",1,TRIvistCallback);
    bump_echo_sub_ = nh.subscribe("turtle1/RTI_bump_echo", 1, bumpEchoCallback);
    aend_echo_sub_ = nh.subscribe("turtle1/RTI_aend_echo",1,aendEchoCallback);
    
    ros::spin();
}
