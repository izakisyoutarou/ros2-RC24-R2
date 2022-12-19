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

  vector<config::LaserPoint> fuse_inliers(PclCloud::Ptr &src_cloud);
  vector<config::LaserPoint> get_inlier(vector<config::LaserPoint> &divided_points);
  void input_points(vector<config::LaserPoint> &points);

};
