#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "config.hpp"

using namespace std;
using PointType = pcl::PointXYZ;
using PclCloud = pcl::PointCloud<PointType>;

class RansacLines{
public:
  RansacLines(){}
  ~RansacLines(){}
  void fuse_inliers(vector<config::LaserPoint> &src_cloud, int &trial_num, double &inlier_dist_threshold, const Pose &estimated, double &odom_to_lidar_x, double &odom_to_lidar_y);
  EstimatedLine get_inlier(vector<config::LaserPoint> &divided_points);
  void set_points(vector<config::LaserPoint> &points, double map_point_x_1, double map_point_x_2, double map_point_y_1, double map_point_y_2, config::LaserPoint &src_point);
  void input_points(EstimatedLine &line);
  bool clear_points(EstimatedLine &estimated_line, int size_threshold, int angle_threshold_min, int angle_threshold_max);
  double calc_diff_angle();
  double LPF_x(double raw);
  double LPF_y(double raw);
  double LPF_yaw(double raw);
  void calc_estimated_diff();
  EstimatedLine get_filtered_rafter_right();
  EstimatedLine get_filtered_rafter_left();
  EstimatedLine get_filtered_rafter_back();
  EstimatedLine get_filtered_fence_centor_right();
  EstimatedLine get_filtered_fence_centor_left();
  EstimatedLine get_filtered_fence_front();
  EstimatedLine get_filtered_fence_right();
  EstimatedLine get_filtered_fence_left();
  vector<config::LaserPoint> get_sum();
  Pose get_estimated_diff();
};
