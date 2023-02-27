#pragma once
#include <vector>
#include <eigen3/Eigen/Dense>
#include "config.hpp"
using namespace std;
using namespace Eigen;

struct LaserPoint{
  double x;
  double y;
  double nx;
  double ny;
  LaserPoint():x(0.0),y(0.0),nx(0.0),ny(0.0){}
};
struct NormalVector{
  double normalize_x;
  double normalize_y;
  NormalVector():normalize_x(0.0),normalize_y(0.0){}
};

class PoseFuser{
public:
  PoseFuser(){}
  ~PoseFuser(){}
  void setup(const double laser_weight, const double odom_weight);
  void init();
  Vector3d fuse_pose(const Vector3d &laser_estimated, const Vector3d &scan_odom_motion, const Vector3d &current_scan_odom, const double dt_scan, const vector<config::LaserPoint> &src_points, const vector<config::LaserPoint> &global_points);

private:
  NormalVector find_correspondence(const vector<config::LaserPoint> &src_points, const vector<config::LaserPoint> &global_points, vector<LaserPoint> &current_points, vector<LaserPoint> &reference_points);
  LaserPoint find_closest_vertical_point(LaserPoint global);
  Matrix3d calc_laser_cov(const Vector3d &laser_estimated, vector<LaserPoint> &current_points, vector<LaserPoint> &reference_points, NormalVector normal_vector, const double laser_weight_);
  double calculate_vertical_distance(const LaserPoint current, const LaserPoint reference, double x, double y, double yaw, NormalVector normal_vector);
  Matrix3d calculate_motion_cov(const Vector3d &scan_odom_motion, const double dt, const double odom_weight_);
  Matrix3d svdInverse(const Matrix3d &A);
  Matrix3d rotate_cov(const Vector3d &laser_estimated, Matrix3d &scan_odom_motion_cov);
  double fuse(const Vector3d &laser_estimated, const Matrix3d &laser_cov, const Vector3d &current_scan_odom, const Matrix3d &rotate_scan_odom_motion_cov, Vector3d &estimated, Matrix3d &fused_cov);


  vector<LaserPoint> current_points;   //対応がとれた現在スキャンの点群
  vector<LaserPoint> reference_points;   //対応がとれた参照スキャンの点群

  double laser_weight_;
  double odom_weight_;
};
