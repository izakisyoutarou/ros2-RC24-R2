#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "config.hpp"

using namespace std;
using namespace Eigen;

class DtectLines{
public:
  DtectLines(){};
  ~DtectLines(){};

  void setup(const int &trial_num, const double &inlier_dist_threshold);
  void init();
  void fuse_inliers(const vector<config::LaserPoint> &src_points, const Vector3d &estimated, const Vector3d &body_to_sensor);

  vector<config::LaserPoint> get_sum();
  Vector3d get_estimated_diff();

private:
  EstimatedLine calc_inliers(vector<config::LaserPoint> &divided_points);
  void get_inliers();
  void devide_points(const vector<config::LaserPoint> &src_points);
  void set_points(vector<config::LaserPoint> &points, const double map_point_x_1, const double map_point_x_2, const double map_point_y_1, const double map_point_y_2, const config::LaserPoint &src_point);
  void input_points(const EstimatedLine &line);
  bool clear_points(EstimatedLine &estimated_line, int size_threshold, int angle_threshold_min, int angle_threshold_max);
  double calc_diff_angle();
  double LPF(const double &raw);
  void calc_estimated_diff(const Vector3d &estimated, const Vector3d &body_to_sensor);
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
  Vector3d estimated_diff = Eigen::Vector3d::Zero();
  int trial_num_;
  double inlier_dist_threshold_;
  double detect_length = 0.0;
  const double distance_threshold=0.5;
  double last_lpf=0;
};
