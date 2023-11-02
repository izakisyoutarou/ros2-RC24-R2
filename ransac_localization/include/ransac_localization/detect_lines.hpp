#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "config.hpp"

using namespace std;
using namespace Eigen;

class DetectLines{
public:
  DetectLines(){};
  ~DetectLines(){};

  void setup(vector<LineData> &_lines, const double &voxel_size, const int &trial_num, const double &inlier_dist_threshold, const double &inlier_length_threshold);
  void init();
  void fuse_inliers(const vector<LaserPoint> &src_points, const Vector3d &laser_pose);

  vector<LaserPoint> get_sum();
  Vector3d get_estimated_diff();

private:
  void ransac_line(LineData &line);
  void get_inliers();
  void devide_points(const vector<LaserPoint> &src_points);
  void input_points(const LineData &line);
  void clear_points(LineData &line);
  double calc_diff_angle();
  void calc_estimated_diff();
  double calc_average(const int &num);

  vector<LineData> lines;
  vector<LaserPoint> sum;
  Vector3d estimated_diff = Vector3d::Zero();
  double voxel_size_;
  int trial_num_;
  double inlier_dist_threshold_;
  double points_num_threshold;
  const double distance_threshold=1.0;
};
