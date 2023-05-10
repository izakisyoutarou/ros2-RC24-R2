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

const double type_1_r = 0.1016/2;
const Vector3d circle_self_right(2.8-6., 2.8-6., type_1_r);
const Vector3d circle_self_center(2.8-6., 6.-6., type_1_r);
const Vector3d circle_self_left(2.8-6., 9.2-6, type_1_r);
const Vector3d circle_opponent_right(9.2-6., 2.8-6., type_1_r);
const Vector3d circle_opponent_left(9.2-6., 9.2-6, type_1_r);
const double ER_map_point_x[4] = {0.05-6., 1.975-6., 5.9875-6., 6.-6.};
const double ER_map_point_y[4] = {0.05-6., 1.975-6, 10.025-6., 11.95-6.};
const double RR_map_point[2] = {4.5-6., 7.5-6.};
const double bridge_width = 0.975;

inline bool uphill_super_correction{true};
constexpr double distance(const double x1, const double y1, const double x2, const double y2) {return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));}
constexpr double get_angle(const double x1, const double y1, const double x2, const double y2){return atan2(y2 - y1, x2 - x1);}
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

inline int get_time_diff(chrono::system_clock::time_point &start){
  return chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now()-start).count();
}
