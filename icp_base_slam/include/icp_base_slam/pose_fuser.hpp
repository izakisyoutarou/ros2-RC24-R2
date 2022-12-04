#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <vector>
#include "config.hpp"
//lidarとオドメトリの推定値を融合する。(imuの融合器はその後作る)
using namespace Eigen;
using namespace std;
using PointType = pcl::PointXYZ;
using PclCloud = pcl::PointCloud<PointType>;

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
private:

public:
  PoseFuser(){}
  ~PoseFuser(){}
  void fuse_pose(const Pose &ndt_estimated, const Pose &scan_odom_motion, const Pose &predict, const double dt_scan, const PclCloud::Ptr current_cloud, const Eigen::Matrix4d transformation_matrix, Pose &estimated, double laser_weight_, double odom_weight_);
  NormalVector find_correspondence(const PclCloud::Ptr current_cloud, const Eigen::Matrix4d transformation_matrix, vector<const LaserPoint*> &current_points, vector<const LaserPoint*> &reference_points);
  LaserPoint* find_closest_vertical_point(LaserPoint global);
  LaserPoint* find_closest_point_in_voxel(PclCloud map_cloud, LaserPoint global, LaserPoint current);
  double calculate_ndt_covariance(const Pose &ndt_estimated, Eigen::Matrix2d &ndt_cov, const Eigen::Matrix4d transformation_matrix, vector<const LaserPoint*> &current_points, vector<const LaserPoint*> &reference_points, NormalVector normal_vector, double laser_weight_);
  double calculate_vertical_distance(const LaserPoint* current, const LaserPoint* reference, double x, double y, double yaw, NormalVector normal_vector);
  void calculate_motion_covariance(const Pose &scan_odom_motion, const double dt, Eigen::Matrix2d &scan_odom_motion_cov, double odom_weight_);
  Eigen::Matrix2d svdInverse(const Matrix2d &A);
  void rotate_covariance(const Pose &ndt_estimated, Eigen::Matrix2d &scan_odom_motion_cov, Eigen::Matrix2d &rotate_scan_odom_motion_cov);
  double calEigen(const Eigen::Matrix3d &cov, double *vals, double *vec1, double *vec2);
  void calEigen2D( double (*mat)[2], double *vals, double *vec1, double *vec2);
  double fuse(const Eigen::Vector2d &mu1, const Eigen::Matrix2d &cv1,  const Eigen::Vector2d &mu2, const Eigen::Matrix2d &cv2, Eigen::Vector2d &mu, Eigen::Matrix2d &cv);
  void print2x2Matrix (const Eigen::Matrix2d & matrix);
};
