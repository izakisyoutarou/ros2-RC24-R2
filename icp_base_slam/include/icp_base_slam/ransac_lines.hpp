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
  void fuse_inliers(PclCloud &src_cloud, int trial_num, double inlier_dist_threshold, const Pose &estimated);
  vector<config::LaserPoint> get_inlier(vector<config::LaserPoint> &divided_points);
  void set_points(vector<config::LaserPoint> &points, double map_point_x_1, double map_point_x_2, double map_point_y_1, double map_point_y_2, PointType src_point, int i);
  void input_points(vector<config::LaserPoint> &points);
  void clear_points(vector<config::LaserPoint> &points, int size_threshold, int angle_threshold_min, int angle_threshold_max);
  vector<config::LaserPoint> get_filtered_rafter_right();
  vector<config::LaserPoint> get_filtered_rafter_left();
  vector<config::LaserPoint> get_filtered_rafter_back();
  vector<config::LaserPoint> get_filtered_fence_centor_right();
  vector<config::LaserPoint> get_filtered_fence_centor_left();
  vector<config::LaserPoint> get_filtered_fence_front();
  vector<config::LaserPoint> get_filtered_fence_right();
  vector<config::LaserPoint> get_filtered_fence_left();
  vector<config::LaserPoint> get_sum();
};
