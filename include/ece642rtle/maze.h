/*
 * Code by Milda Zizyte (milda@cmu.edu)
 * and Phil Koopman (koopman@cmu.edu)
 */

#include <fstream>
#include <iostream>
#include <string>
#include <tuple>
#include <ros/ros.h>
#include <ros/package.h>

namespace ece642rtle
{
  
  class Maze
{
  
 public:
  Maze();
  
  struct LINE_SEGMENT {
    int x1;
    int y1;
    int x2;
    int y2;
  } line_segment;
  
  LINE_SEGMENT* get_line_segments();
  int get_line_segments_length();
  bool is_in_line_segments(int in_x1, int in_y1, int in_x2, int in_y2);
  LINE_SEGMENT get_start();
  LINE_SEGMENT get_end();
  std::tuple<int, int> get_start_square();
  bool at_end(float x1, float y1);
  int get_initial_rotation();
  
 private:
  LINE_SEGMENT* line_segments_;
  int line_segments_len_;
  LINE_SEGMENT start_;
  LINE_SEGMENT end_;
  int initial_rotation_;
  bool points_between(int i1, int i2, int j1, int j2);
};
}
