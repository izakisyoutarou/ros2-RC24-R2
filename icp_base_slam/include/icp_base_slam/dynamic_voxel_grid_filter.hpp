#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <random>
#include <iomanip>
#include <chrono>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "config.hpp"

using namespace std;
using PointType = pcl::PointXYZ;
using PclCloud = pcl::PointCloud<PointType>;

class DynamicVoxelGridFilter{
public:
  DynamicVoxelGridFilter(){}
  ~DynamicVoxelGridFilter(){}
  vector<config::LaserPoint> variable_voxel(vector<config::LaserPoint> &points);
  config::LaserPoint calc_centroid(vector<config::LaserPoint> &points);
  config::LaserPoint calc_max(vector<config::LaserPoint> &points);
  config::LaserPoint calc_min(vector<config::LaserPoint> &points);
};
