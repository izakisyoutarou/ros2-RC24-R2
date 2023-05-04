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
  DetectCircles();
  ~DetectCircles(){};
  Vector3d get_diff_pose(const vector<LaserPoint> &src_points);
  void init();
  Vector3d super_correction();


private:
  struct CirclesData{
    vector<LaserPoint> points;
    double rate=0.0;
  };

  Vector3d calc_diff(const Vector3d &circle, const int &num);
  Vector3d get_best_circle(CirclesData &circles_data);
  void devide_points(const vector<LaserPoint> &src_points);
  void create_voxel(vector<LaserPoint> &points, const Vector3d &circle, const vector<LaserPoint> &src_points);
  Vector3d calc_diff_pose();

  Vector3d estimated_diff;
  vector<Vector3d> circles;
  vector<CirclesData> circles_datas;
  CirclesData circles_data;

};
