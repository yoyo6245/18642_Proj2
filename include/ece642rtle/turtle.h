/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Modified by Milda Zizyte (milda@cmu.edu)
 * and Philip Koopman (koopman@cmu.edu)
 */

#ifndef TURTLESIM_TURTLE_H
#define TURTLESIM_TURTLE_H

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

#include <ece642rtle/timeInt8.h>
#include <std_msgs/Empty.h>
#include <ece642rtle/RTIbump.h>
#include <ece642rtle/RTIatend.h>
#include <ece642rtle/PoseOrntBundle.h>
#include <QColor>
#include <QRgb>
#include <boost/bind.hpp>
#endif

#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPointF>


#include "maze.h"

#define SCREEN_TURTLE_WIDTH 12
#define TURTLE_SCALE 50

#define PI 3.1415926535

namespace ece642rtle
{

class Turtle
{
public:
  Turtle(const ros::NodeHandle& nh,
	 const QImage& turtle_image,
	 const QPointF& pos,
	 float orient,
	 const Maze& turtle_maze);
  
  bool update(double dt, QPainter& path_painter, const QImage& path_image, qreal canvas_width, qreal canvas_height);
  void paint(QPainter& painter);
  
  void rotateImage(int rotation);
  int orientation_;
  int current_square_visits_;
  // Background/0, 1, 2, 3, 4+
  const static int bg_colors[5][3];
  
 private:
  QColor lookUpColor(int visits);
  void RTIreqCallback(const std_msgs::Empty::ConstPtr&);
  void RTIpobCallback(const ece642rtle::PoseOrntBundle::ConstPtr& pob);
  void TRIvisitCallback(const ece642rtle::timeInt8::ConstPtr& v);
  bool RTIbumpCallback(ece642rtle::RTIbump::Request&, ece642rtle::RTIbump::Response&);
  bool RTIatendCallback(ece642rtle::RTIatend::Request&, ece642rtle::RTIatend::Response&);
  
  void rotateImage();
  
  ros::NodeHandle nh_;
  
  QImage turtle_image_;
  QImage turtle_rotated_image_;
  
  Maze turtle_maze_;
  
  QPointF pos_;
  
  bool RTI_received_;

  ros::Subscriber RTI_req_sub_;
  ros::Subscriber RTI_pob_sub_;
  ros::Subscriber TRI_vis_sub_;
  ros::Publisher RTI_pub_;
  ros::ServiceServer RTI_at_end_srv_;
  ros::ServiceServer RTI_bump_srv_;
  
  ros::WallTime last_command_time_;
  
  float turtle_scale_;
};
 typedef boost::shared_ptr<Turtle> TurtlePtr;
}

#endif
