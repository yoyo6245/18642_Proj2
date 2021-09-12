/*
 * Code  by Milda Zizyte (milda@cmu.edu)
 * and Philip Koopman (koopman@cmu.edu)
 */

#include "ece642rtle/maze.h"

#define WIDTH 12

namespace ece642rtle
{
Maze::Maze() {
  // Start the count at 4 because of implicit outer walls
  line_segments_len_ = 4;
  initial_rotation_ = -1;
  ros::NodeHandle n;
  std::string filename;
  // Assume all mazes are in top ece642rtle/ directory.
  // The ROS param that is passed to this file does not contain a path
  n.param("/maze_file", filename, std::string("m1.maze"));
  filename = ros::package::getPath("ece642rtle") + "/" + filename;
  ROS_INFO("Using mazefile %s", filename.c_str());
  std::string line;
  std::ifstream myfile(filename);
  
  // Count the number of lines in the file
  if(myfile.is_open()) {
    while(getline(myfile, line)) {
      if (line.compare("RAND_DIR") != 0) {
	line_segments_len_++;
      }
    }
    myfile.close();
  }
  
  line_segments_ = new LINE_SEGMENT[line_segments_len_];
  
  int p = 0;
  std::ifstream myfile2(filename);
  if(myfile2.is_open()) {
    while(getline(myfile2, line)) {
      // Random initial rotation
      if (line.compare("RAND_DIR") == 0) {
	srand(time(NULL));
	initial_rotation_ = (rand() % 4);
	ROS_INFO("Initial rotation is %d", initial_rotation_);
      } else {
	// Segment the line by commas
	LINE_SEGMENT l;
	int stri = 0;
	int old_stri = 0;
	int arri = 0;
	int* l_arr = new int[4];
	while(line[stri] != '\0') {
	  // Reach a comma; grab the substring between old_stri and stri
	  if(line[stri] == ',') {
	    l_arr[arri] = std::stoi(line.substr(old_stri, stri));
	    old_stri = stri+1;
	    arri++;
	  }
	  stri++;
	}
	l_arr[arri] = std::stoi(line.substr(old_stri, stri));
	l.x1 = l_arr[0];
	l.y1 = l_arr[1];
	l.x2 = l_arr[2];
	l.y2 = l_arr[3];
	line_segments_[p] = l;
	p++;
      }
    }
    myfile2.close();
  }
 
  // Outer walls
  line_segments_[p++] = { 0, 0, 0, 12 }; 
  line_segments_[p++] = { 0, 12, 12, 12 }; 
  line_segments_[p++] = { 12, 12, 12, 0 };
  line_segments_[p++] = { 12, 0, 0, 0 };
  
  // Start and end top-left and bottom-right corners
  start_ = {line_segments_[0].x1, line_segments_[0].y1, line_segments_[0].x2, line_segments_[0].y2};
  end_   = {line_segments_[1].x1, line_segments_[1].y1, line_segments_[1].x2, line_segments_[1].y2};
  
}
  
  Maze::LINE_SEGMENT* Maze::get_line_segments()
  {
    return line_segments_;
  }  
  
  int Maze::get_line_segments_length()
  {
    return line_segments_len_;
  }
  
  Maze::LINE_SEGMENT Maze::get_start() {
    return start_;
  }
  
  Maze::LINE_SEGMENT Maze::get_end() {
    return end_;
  }
  
  std::tuple<int, int> Maze::get_start_square() {
    return std::make_tuple(start_.x1, start_.y1);
  }
  
  bool Maze::at_end(float x1, float y1) {
    return (x1 > end_.x1 && y1 > end_.y1 && x1 < end_.x2 && y1 < end_.y2);
    
  }
  
  int Maze::get_initial_rotation() {
    return initial_rotation_;
  }

  /* 
   *Returns true if a1 and a2 fall between b1 and b2 (inclusive), false otherwise
   * Assumption: j1 < j2
   */
  bool Maze::points_between(int i1, int i2, int j1, int j2) {
    return i1 >= j1 && i1 <= j2 && i2 >= j1 && i2 <= j2;
  }

  /*
   * Returns true if the line segment between (in_x1,in_y1) and (in_x2,in_y2)
   * is in the maze
   * Assumption: length of line segment is 1
   */
  bool Maze::is_in_line_segments(int in_x1, int in_y1, int in_x2, int in_y2)
  {
    // Convert in_x1, in_y1, in_x2, in_y2 to a canonical form:
    // either x- or y- coords are the same. This is the "a" variable
    // other two coords are "b1" and "b2" (order does not matter)
    int a, b1, b2;
    bool x_same;
    if (in_x1 == in_x2) {
      a = in_x1;
      b1 = in_y1;
      b2 = in_y2;
      x_same = true;
    } else if (in_y1 == in_y2) {
      a = in_y1;
      b1 = in_x1;
      b2 = in_x2;
      x_same = false;
    } else {
      return false;
    }

    // Check each maze wall segment to see if it encompasses the input endpoints
    for(int i = 2; i < line_segments_len_; i++) {
      LINE_SEGMENT seg = line_segments_[i];
    
    // convert seg to canonical form: c1 and c2 correspond to the dimension where the input coordinates are the same
    // (this is to check that "a" defines the same axis as c1 and c2, so that the b's and d's lie on this same axis)
    // d1 and d2 are the endpoints on this axis, with d1 < d2
    int c1,c2,d1,d2;
    if (x_same) {
      c1 = seg.x1;
      c2 = seg.x2;
      d1 = seg.y1;
      d2 = seg.y2;
    } else {
      c1 = seg.y1;
      c2 = seg.y2;
      d1 = seg.x1;
      d2 = seg.x2;
    }
    if (d2 < d1) {
      int t = d1;
      d1 = d2;
      d2 = t;
    }
    
    // If a, c1, c2 define same axis, then we check that b1 and b2 fall between d1 and d2
    if (a == c1 && a == c2) {
      if (points_between(b1, b2, d1, d2)) {
	return true;
      }
    }
  }
  return false;
}
}
