#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace Eigen;

struct LaserPoint{
  double x=0.0;
  double y=0.0;
};

const bool use_simulator{true};

const double ER_map_point_x[4] = {0.05-6., 1.975-6., 5.9875-6., 6.-6.};
const double ER_map_point_y[4] = {0.05-6., 1.975-6, 10.025-6., 11.95-6.};
const double RR_map_point[4] = {2.6-6., 4.5-6., 7.5-6., 9.4-6.};
const double bridge_width = 0.975;
constexpr double radToDeg(const double rad){ return rad*180/M_PI; };
constexpr double normalize_yaw(double yaw){
  if (yaw < -M_PI) yaw += 2*M_PI;
  else if (yaw >= M_PI) yaw -= 2*M_PI;
  return yaw;
}

// 点群を並進・回転させる
constexpr LaserPoint rotate(LaserPoint point, double theta){
  LaserPoint p;
  p.x = point.x * cos(theta) - point.y * sin(theta);
  p.y = point.x * sin(theta) + point.y * cos(theta);
  return p;
}

inline vector<LaserPoint> transform(const vector<LaserPoint> &points, const Vector3d &pose) {
  vector<LaserPoint> transformed_points;
  for (const auto& point : points) {
    // 並進
    LaserPoint p = point;
    p.x += pose[0];
    p.y += pose[1];
    // 回転
    p = rotate(p, pose[2]);
    transformed_points.push_back(p);
  }
  return transformed_points;
}
