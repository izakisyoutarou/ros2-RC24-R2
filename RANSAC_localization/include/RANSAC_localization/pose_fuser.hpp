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
  Vector3d fuse_pose(const Vector3d &ransac_estimated, const Vector3d &scan_odom_motion, const Vector3d &current_scan_odom, const double dt_scan, const vector<config::LaserPoint> &src_points, const vector<config::LaserPoint> &global_points);

private:
  NormalVector find_correspondence(const vector<config::LaserPoint> &src_points, const vector<config::LaserPoint> &global_points, vector<LaserPoint> &current_points, vector<LaserPoint> &reference_points);
  LaserPoint find_closest_vertical_point(LaserPoint global);
  Eigen::Matrix2d calculate_ransac_covariance(const Vector3d &ransac_estimated, vector<LaserPoint> &current_points, vector<LaserPoint> &reference_points, NormalVector normal_vector, const double laser_weight_);
  double calculate_vertical_distance(const LaserPoint current, const LaserPoint reference, double x, double y, double yaw, NormalVector normal_vector);
  Eigen::Matrix2d calculate_motion_covariance(const Vector3d &scan_odom_motion, const double dt, const double odom_weight_);
  Eigen::Matrix2d svdInverse(const Matrix2d &A);
  Eigen::Matrix2d rotate_covariance(const Vector3d &ransac_estimated, Eigen::Matrix2d &scan_odom_motion_cov);
  double fuse(const Eigen::Vector2d &mu1, const Eigen::Matrix2d &cv1, const Eigen::Vector2d &mu2, const Eigen::Matrix2d &cv2, Eigen::Vector2d &mu, Eigen::Matrix2d &cv);

  vector<LaserPoint> current_points;   //対応がとれた現在スキャンの点群
  vector<LaserPoint> reference_points;   //対応がとれた参照スキャンの点群

  double laser_weight_;
  double odom_weight_;
};
