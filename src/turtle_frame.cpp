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

#include "ece642rtle/turtle_frame.h"

#include <QPointF>

#include <ros/package.h>
#include <cstdlib>
#include <ctime>

namespace ece642rtle
{

TurtleFrame::TurtleFrame(QWidget* parent, Qt::WindowFlags f)
    : QFrame(parent, f)
    , path_image_((SCREEN_TURTLE_WIDTH + 1) * TURTLE_SCALE,
                  (SCREEN_TURTLE_WIDTH + 2) * TURTLE_SCALE,
                  QImage::Format_ARGB32)
    , path_painter_(&path_image_)
    , frame_count_(0)
    , id_counter_(0)
    , turtle_maze_()
{
    setFixedSize((SCREEN_TURTLE_WIDTH + 1) * TURTLE_SCALE, (SCREEN_TURTLE_WIDTH + 2) * TURTLE_SCALE);
    setWindowTitle("642rtle Sim");

    srand(time(NULL));

    update_timer_ = new QTimer(this);
    update_timer_->setInterval(16);
    update_timer_->start();

    connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

    nh_.setParam("background_r", Turtle::bg_colors[0][0]);
    nh_.setParam("background_g", Turtle::bg_colors[0][1]);
    nh_.setParam("background_b", Turtle::bg_colors[0][2]);

    QVector<QString> turtles;
    turtles.append("box-turtle.png");
    turtles.append("robot-turtle.png");
    turtles.append("sea-turtle.png");
    turtles.append("diamondback.png");
    turtles.append("electric.png");
    turtles.append("fuerte.png");
    turtles.append("groovy.png");
    turtles.append("hydro.svg");
    turtles.append("indigo.svg");
    turtles.append("jade.png");
    turtles.append("kinetic.png");
    turtles.append("lunar.png");

    QString images_path = (ros::package::getPath("ece642rtle") + "/images/").c_str();
    for(int i = 0; i < turtles.size(); ++i) {
        QImage img;
        img.load(images_path + turtles[i]);
        turtle_images_.append(img);
    }

    clear();

    clear_srv_ = nh_.advertiseService("clear", &TurtleFrame::clearCallback, this);
    reset_srv_ = nh_.advertiseService("reset", &TurtleFrame::resetCallback, this);
    spawn_srv_ = nh_.advertiseService("spawn", &TurtleFrame::spawnCallback, this);
    kill_srv_ = nh_.advertiseService("kill", &TurtleFrame::killCallback, this);

    ROS_INFO("Starting ece642rtle with node name %s", ros::this_node::getName().c_str());
    float angle = 0;
    if (turtle_maze_.get_initial_rotation() != -1) {
      angle = 90.0 * turtle_maze_.get_initial_rotation();
    }
    spawnTurtle("",
                std::get<0>(turtle_maze_.get_start_square()),
                std::get<1>(turtle_maze_.get_start_square()),
                angle);

    // spawn all available turtle types
    if(false) {
        for(int index = 0; index < turtles.size(); ++index) {
            QString name = turtles[index];
            name = name.split(".").first();
            name.replace(QString("-"), QString(""));
            spawnTurtle(name.toStdString(), 1.0 + 1.5 * (index % 7), 1.0 + 1.5 * (index / 7), PI / 2.0, index);
        }
    }
}

TurtleFrame::~TurtleFrame()
{
    delete update_timer_;
}

bool TurtleFrame::spawnCallback(ece642rtle::Spawn::Request& req, ece642rtle::Spawn::Response& res)
{
    std::string name = spawnTurtle(req.name, req.x, req.y, req.theta);
    if(name.empty()) {
        ROS_ERROR("A turtled named [%s] already exists", req.name.c_str());
        return false;
    }

    res.name = name;

    return true;
}

bool TurtleFrame::killCallback(ece642rtle::Kill::Request& req, ece642rtle::Kill::Response&)
{
    M_Turtle::iterator it = turtles_.find(req.name);
    if(it == turtles_.end()) {
        ROS_ERROR("Tried to kill turtle [%s], which does not exist", req.name.c_str());
        return false;
    }

    turtles_.erase(it);
    update();

    return true;
}

bool TurtleFrame::hasTurtle(const std::string& name)
{
    return turtles_.find(name) != turtles_.end();
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle)
{
    return spawnTurtle(name, x, y, angle, rand() % turtle_images_.size());
}

std::string TurtleFrame::spawnTurtle(const std::string& name, float x, float y, float angle, size_t index)
{
    std::string real_name = name;
    if(real_name.empty()) {
        do {
            std::stringstream ss;
            ss << "turtle" << ++id_counter_;
            real_name = ss.str();
        } while(hasTurtle(real_name));
    } else {
        if(hasTurtle(real_name)) {
            return "";
        }
    }

    TurtlePtr t(new Turtle(ros::NodeHandle(real_name), turtle_images_[index], QPointF(x, y), angle, turtle_maze_));
    turtles_[real_name] = t;
    update();

    ROS_INFO("Spawning turtle [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

    return real_name;
}

  /*
   * Draw all the background elements
   * Sorry that there's so much code here... -MZ
   */
void TurtleFrame::clear()
{
  // Background
  int r = Turtle::bg_colors[0][0];
  int g = Turtle::bg_colors[0][1];
  int b = Turtle::bg_colors[0][2];
  nh_.param("background_r", r, r);
  nh_.param("background_g", g, g);
  nh_.param("background_b", b, b);
  path_image_.fill(qRgb(r, g, b));

  // Visit count legend
  for (int ii = 0; ii < 5; ii++) {
    QBrush legend_brush(QColor(Turtle::bg_colors[ii][0], Turtle::bg_colors[ii][1], Turtle::bg_colors[ii][2]));
    QPen legend_pen(QColor(0,0,0));
    legend_pen.setWidth(2);
    QRectF legend_rect(TURTLE_SCALE/2 + (ii*2.4)*TURTLE_SCALE, (SCREEN_TURTLE_WIDTH + 1) * TURTLE_SCALE, TURTLE_SCALE/2, TURTLE_SCALE/2);
    path_painter_.setPen(legend_pen);
    path_painter_.drawRect(legend_rect);
    path_painter_.fillRect(legend_rect, legend_brush);
    QString legend_text;
    if (ii == 0) {
      legend_text = QString("Unmarked/0");
    } else if (ii == 1) {
      legend_text = QString("1 visit");
    } else if (ii == 4) {
      legend_text = QString("4+ visits");
    } else {
      legend_text = QString((std::to_string(ii) + " visits").c_str());
    }
    path_painter_.drawText(TURTLE_SCALE/2 + (ii*2.4 + .6)*TURTLE_SCALE, (SCREEN_TURTLE_WIDTH + 1.3) * TURTLE_SCALE, legend_text);
  }

  // Draw faint grid
  QPen test_pen(QColor(0x80, 0x80, 0xff));
  test_pen.setWidth(1);
  path_painter_.setPen(test_pen);
  // Vertical and horizontal line every TURTLE_SCALE pixels
  for(int i = 0; i <= SCREEN_TURTLE_WIDTH; i++) {
    path_painter_.drawLine(TURTLE_SCALE / 2 + TURTLE_SCALE * i,
			   TURTLE_SCALE / 2,
			   TURTLE_SCALE / 2 + TURTLE_SCALE * i,
			   TURTLE_SCALE / 2 + TURTLE_SCALE * SCREEN_TURTLE_WIDTH);
    path_painter_.drawLine(TURTLE_SCALE / 2,
			   TURTLE_SCALE / 2 + TURTLE_SCALE * i,
			   TURTLE_SCALE / 2 + TURTLE_SCALE * SCREEN_TURTLE_WIDTH,
			   TURTLE_SCALE / 2 + TURTLE_SCALE * i);
  }

  // Maze walls
  test_pen.setColor(QColor(0x00, 0x00, 0x00));
  test_pen.setWidth(5);
  path_painter_.setPen(test_pen);
  int segment_number = turtle_maze_.get_line_segments_length();
  Maze::LINE_SEGMENT* line_segments = turtle_maze_.get_line_segments();
  // Convert maze coordinates by multiplying them by TURTLE_SCALE
  for(int i = 0; i < segment_number; i++) {
    path_painter_.drawLine(line_segments[i].x1 * TURTLE_SCALE + TURTLE_SCALE / 2,
			   line_segments[i].y1 * TURTLE_SCALE + TURTLE_SCALE / 2,
			   line_segments[i].x2 * TURTLE_SCALE + TURTLE_SCALE / 2,
			   line_segments[i].y2 * TURTLE_SCALE + TURTLE_SCALE / 2);
  }
  
  // start square (green X)
  test_pen.setColor(QColor(0x20, 0xff, 0x20));
  path_painter_.setPen(test_pen);
  Maze::LINE_SEGMENT start_segment = turtle_maze_.get_start();
  // top-left to bottom-right
  path_painter_.drawLine(start_segment.x1 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 start_segment.y1 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 start_segment.x2 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 start_segment.y2 * TURTLE_SCALE + TURTLE_SCALE / 2);
  // top-right to bottom-left
  path_painter_.drawLine(start_segment.x2 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 start_segment.y1 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 start_segment.x1 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 start_segment.y2 * TURTLE_SCALE + TURTLE_SCALE / 2);
  
  // end square (red X)
  test_pen.setColor(QColor(0xff, 0x20, 0x20));
  path_painter_.setPen(test_pen);
  Maze::LINE_SEGMENT end_segment = turtle_maze_.get_end();
  // top-left to bottom-right
  path_painter_.drawLine(end_segment.x1 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 end_segment.y1 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 end_segment.x2 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 end_segment.y2 * TURTLE_SCALE + TURTLE_SCALE / 2);
  // top-right to bottom-left
  path_painter_.drawLine(end_segment.x2 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 end_segment.y1 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 end_segment.x1 * TURTLE_SCALE + TURTLE_SCALE / 2,
			 end_segment.y2 * TURTLE_SCALE + TURTLE_SCALE / 2);
  update();
}
  
void TurtleFrame::onUpdate()
{
  ros::spinOnce();
  
  updateTurtles();
  
  if(!ros::ok()) {
    close();
  }
}
  
  void TurtleFrame::paintEvent(QPaintEvent*)
  {
    
    QPainter painter(this);
    
    painter.drawImage(QPoint(0, 0), path_image_);
    
    M_Turtle::iterator it = turtles_.begin();
    M_Turtle::iterator end = turtles_.end();
    for(; it != end; ++it) {
      it->second->paint(painter);
    }
  }
  
  void TurtleFrame::updateTurtles()
  {
    if(last_turtle_update_.isZero()) {
      last_turtle_update_ = ros::WallTime::now();
      return;
    }
    
    bool modified = false;
    M_Turtle::iterator it = turtles_.begin();
    M_Turtle::iterator end = turtles_.end();
    for(; it != end; ++it) {
      modified |= it->second->update(
				     0.001 * update_timer_->interval(), path_painter_, path_image_, SCREEN_TURTLE_WIDTH, SCREEN_TURTLE_WIDTH);
    }
    if(true) // modified)
      {
        update();
      }
    
    ++frame_count_;
  }
  
  bool TurtleFrame::clearCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    ROS_INFO("Clearing ece642rtle.");
    clear();
    return true;
  }
  
  bool TurtleFrame::resetCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    ROS_INFO("Resetting ece642rtle.");
    turtles_.clear();
    id_counter_ = 0;
    float angle = 0;
    if (turtle_maze_.get_initial_rotation() != -1) {
      angle = 90.0 * turtle_maze_.get_initial_rotation();
    }
    spawnTurtle("",
                std::get<0>(turtle_maze_.get_start_square()),
                std::get<1>(turtle_maze_.get_start_square()),
                angle); // SCREEN_TURTLE_WIDTH/2, SCREEN_TURTLE_WIDTH/2, 0);
    clear();
    return true;
  }
}
