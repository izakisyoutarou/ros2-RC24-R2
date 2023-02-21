#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>

#include "config.hpp"

using namespace std;

class DtectLines{
public:
  DtectLines(){};
  ~DtectLines(){};

  void setup(const int &trial_num, const double &inlier_dist_threshold);
  void init();
  void fuse_inliers(const vector<config::LaserPoint> &src_points, const Pose &estimated, const double &odom_to_lidar_x, const double &odom_to_lidar_y);

  vector<config::LaserPoint> get_sum();
  Pose get_estimated_diff();

private:
  EstimatedLine calc_inliers(vector<config::LaserPoint> &divided_points);
  void get_inliers();
  void devide_points(const vector<config::LaserPoint> &src_points);
  void set_points(vector<config::LaserPoint> &points, const double map_point_x_1, const double map_point_x_2, const double map_point_y_1, const double map_point_y_2, const config::LaserPoint &src_point);
  void input_points(const EstimatedLine &line);
  bool clear_points(EstimatedLine &estimated_line, int size_threshold, int angle_threshold_min, int angle_threshold_max);
  double calc_diff_angle();
  double LPF(const double &raw);
  void calc_estimated_diff(const Pose &estimated, const double &odom_to_lidar_x, const double &odom_to_lidar_y);
  /* linesの順番
  0:rafter_right
  1:fence_right
  2:fence_left
  3:rafter_left
  4:rafter_back
  5:fence_front
  6:fence_centor_right
  7:fence_centor_left
  */
  vector<vector<config::LaserPoint>> lines;
  vector<EstimatedLine> lines_;
  vector<config::LaserPoint> sum;
  EstimatedLine inlier;
  Pose estimated_diff;
  int trial_num_;
  double inlier_dist_threshold_;
  double detect_length = 0.0;
  const double distance_threshold=0.8;
  double last_lpf=0;
  chrono::system_clock::time_point time_start, time_end;
};
