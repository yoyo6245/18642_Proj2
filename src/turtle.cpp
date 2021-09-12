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

#include "ece642rtle/turtle.h"

namespace ece642rtle
{

  const int Turtle::bg_colors[5][3] = {{86, 180, 233}, {213, 94, 0}, {0,158,115}, {230, 159, 0}, {204, 121, 167}};
  
  Turtle::Turtle(const ros::NodeHandle& nh,
		 const QImage& turtle_image,
		 const QPointF& pos,
		 float orient,
		 const Maze& turtle_maze)
    : nh_(nh)
    , turtle_image_(turtle_image)
    , turtle_maze_(turtle_maze)
    , pos_(pos)
    , orientation_((int)(orient/90))
    , RTI_received_(false)
    , current_square_visits_(-1)
  {
    
    RTI_req_sub_ = nh_.subscribe("TRI_req", 1, &Turtle::RTIreqCallback, this);
    RTI_pob_sub_ = nh_.subscribe("TRI_pob", 1, &Turtle::RTIpobCallback, this);
    TRI_vis_sub_ = nh_.subscribe("TRI_vist", 1, &Turtle::TRIvisitCallback, this);
    RTI_pub_ = nh_.advertise<ece642rtle::PoseOrntBundle>("RTI_pob", 1);
    RTI_at_end_srv_ = nh_.advertiseService("RTI_at_end", &Turtle::RTIatendCallback, this);
    RTI_bump_srv_ = nh_.advertiseService("RTI_bump", &Turtle::RTIbumpCallback, this);
    rotateImage();
  }
  
  
  void Turtle::RTIreqCallback(const std_msgs::Empty::ConstPtr&) {
    RTI_received_ = true;
  }

  void Turtle::RTIpobCallback(const ece642rtle::PoseOrntBundle::ConstPtr& pob) {
    pos_.setX(pob->x);
    pos_.setY(pob->y);
    orientation_ = pob->o;
    rotateImage(orientation_);
    RTI_received_ = true;
  }

  void Turtle::TRIvisitCallback(const ece642rtle::timeInt8::ConstPtr& v) {
    current_square_visits_ = v->data;
  }
  
  bool Turtle::RTIbumpCallback(ece642rtle::RTIbump::Request& req, ece642rtle::RTIbump::Response& res) {
    res.bump = turtle_maze_.is_in_line_segments(req.x1, req.y1, req.x2, req.y2);
    return true;
  }
  
  bool Turtle::RTIatendCallback(ece642rtle::RTIatend::Request& req, ece642rtle::RTIatend::Response& res) {
    res.atend = turtle_maze_.at_end(((float)(req.x)+0.5), ((float)(req.y)+0.5) ); 
  }
  
  void Turtle::rotateImage()
  {
    QTransform transform;
    transform.rotate(90 * (orientation_-1));
    turtle_rotated_image_ = turtle_image_.transformed(transform);
  }
  
  void Turtle::rotateImage(int rotation)
  {
    QTransform transform;
    transform.rotate(90 * (rotation-1));
    turtle_rotated_image_ = turtle_image_.transformed(transform);
  }
  
  QColor Turtle::lookUpColor(int visits) {
    QColor rval;
    if (visits > 4) {
      visits = 4;
    }
    rval = QColor(Turtle::bg_colors[visits][0], Turtle::bg_colors[visits][1], Turtle::bg_colors[visits][2]);
    return rval;
  }
  
  bool Turtle::update(double dt, QPainter& path_painter, const QImage& path_image, qreal canvas_width, qreal canvas_height)
  {
    if (!RTI_received_) {
      return false;
    }
    
    // Clamp to screen size
    if(pos_.x() < 0 || pos_.x() >= SCREEN_TURTLE_WIDTH || pos_.y() < 0 || pos_.y() >= SCREEN_TURTLE_WIDTH) {
      ROS_WARN("Oh no! I hit the wall! (Clamping from [x=%f, y=%f])", pos_.x(), pos_.y());
    }
    
    pos_.setX(std::min(std::max(static_cast<double>(pos_.x()), 0.0), static_cast<double>(SCREEN_TURTLE_WIDTH - 1)));
    pos_.setY(std::min(std::max(static_cast<double>(pos_.y()), 0.0), static_cast<double>(SCREEN_TURTLE_WIDTH - 1)));
    
    if (current_square_visits_ >= 0) {
      QBrush b(lookUpColor(current_square_visits_));
      QRectF bg((pos_ * TURTLE_SCALE) + QPointF(.7*TURTLE_SCALE, .7*TURTLE_SCALE), QSize(.6*TURTLE_SCALE, .6*TURTLE_SCALE));
      path_painter.fillRect(bg, b);
      path_painter.setPen(QPen(QColor(0,0,0)));
      path_painter.drawText((pos_ * TURTLE_SCALE) + QPointF(.8*TURTLE_SCALE, 1.1*TURTLE_SCALE), std::to_string(current_square_visits_).c_str());
      current_square_visits_ = -1;
    }
    
    // Publish pose of the turtle
    PoseOrntBundle pob;
    pob.x = pos_.x();
    pob.y = pos_.y();
    pob.o = orientation_;
    pob.t = ros::Time::now();
    RTI_pub_.publish(pob);
    
    RTI_received_ = false;
    return true;
  }
  
  void Turtle::paint(QPainter& painter)
  {
    
    QPointF p = pos_ * TURTLE_SCALE;
    p.rx() += TURTLE_SCALE - 0.35 * turtle_rotated_image_.width();
    p.ry() += TURTLE_SCALE - 0.35 * turtle_rotated_image_.height();
    QRectF r(p, QSize(turtle_rotated_image_.width()*0.7, turtle_rotated_image_.height()*0.7) );
    
    painter.drawImage(r, turtle_rotated_image_);
  }
  
}
