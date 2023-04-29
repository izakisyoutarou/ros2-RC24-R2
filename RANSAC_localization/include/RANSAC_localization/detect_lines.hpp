#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "config.hpp"

using namespace std;
using namespace Eigen;

struct EstimatedLine{
  double angle=0.0;
  vector<LaserPoint> points;
};

class DtectLines{
public:
  DtectLines(){};
  ~DtectLines(){};

  void setup(const string &robot_type, const double &voxel_size, const int &trial_num, const double &inlier_dist_threshold, const double &inlier_length_threshold);
  void init();
  void fuse_inliers(const vector<LaserPoint> &src_points);

  vector<LaserPoint> get_sum();
  Vector3d get_estimated_diff();

private:
  EstimatedLine calc_inliers(vector<LaserPoint> &divided_points);
  void get_inliers();
  void devide_points(const vector<LaserPoint> &src_points);
  void set_points(vector<LaserPoint> &points, const double map_point_x_1, const double map_point_x_2, const double map_point_y_1, const double map_point_y_2, const LaserPoint &src_point);
  void input_points(const EstimatedLine &line);
  bool clear_points(EstimatedLine &estimated_line, int angle_threshold_min, int angle_threshold_max);
  double calc_diff_angle();
  double LPF(const double &raw);
  void calc_estimated_diff();
  void calc_tracking_diff(const int &num);
  double check_tracking();
  double calc_average(const int &num);

  vector<vector<LaserPoint>> lines;
  /* linesの順番
  0:rafter_right
  1:fence_right
  2:fence_left
  3:rafter_left
  4:rafter_back
  5:fence_front
  6:fence_centor_right
  7:fence_centor_left*/
  vector<EstimatedLine> lines_;
  vector<LaserPoint> sum;
  EstimatedLine inlier;
  Vector3d estimated_diff = Vector3d::Zero();
  string robot_type_;
  double voxel_size_;
  int trial_num_;
  double inlier_dist_threshold_;
  double points_num_threshold;
  double detect_length = 0.0;
  const double distance_threshold=1.0;
  double last_lpf=0;
  bool is_tracking_rafter_left{false};
  bool is_tracking_rafter_right{false};

  chrono::system_clock::time_point detect_rafter_left_time, detect_rafter_right_time;
};
