#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <chrono>

#include "config.hpp"

using namespace std;

class DetectCircles{
public:
  DetectCircles(){};
  ~DetectCircles(){};
  void calc_pose(const vector<LaserPoint> &src_points);
private:
  void init();
  double distance(LaserPoint point, double x2, double y2) {
    return sqrt((x2 - point.x) * (x2 - point.x) + (y2 - point.y) * (y2 - point.y));
  }
  Circle get_inliers(vector<LaserPoint> &points);
  void devide_points(const vector<LaserPoint> &src_points);
  void create_voxel(const LaserPoint &lp, vector<LaserPoint> &points, const Circle &circle);

  vector<vector<LaserPoint>> circles_points;
};
