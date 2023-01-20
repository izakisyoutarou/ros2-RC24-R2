#pragma once

#include <iostream>
#include <vector>
#include <cmath>
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
  void input_points(vector<config::LaserPoint> &points);
  vector<config::LaserPoint> get_filtered_rafter_right();
  vector<config::LaserPoint> get_filtered_rafter_left();
  vector<config::LaserPoint> get_filtered_rafter_back();
  vector<config::LaserPoint> get_filtered_fence_centor();
  vector<config::LaserPoint> get_filtered_fence_front();
  vector<config::LaserPoint> get_filtered_fence_right();
  vector<config::LaserPoint> get_filtered_fence_left();
  vector<config::LaserPoint> get_sum();
};
