#pragma once
#include "config.hpp"
#include "icp_base_slam/icp_base_slam.hpp"
//lidarとオドメトリの推定値を融合する。(imuの融合器はその後作る)
class PoseFuser{
public:
  PoseFuser(){}
  struct LaserPoint{
    double x;
    double y;
    LaserPoint():x(0.0),y(0.0){}

    void operator=(const LaserPoint &other) {
    x = other.x;
    y = other.y;
    }

    LaserPoint operator-(const LaserPoint &other){
      LaserPoint laser_point;
      laser_point.x = x - other.x;
      laser_point.y = y - other.y;
      return laser_point;
    }

    LaserPoint operator+(const LaserPoint &other){
      LaserPoint laser_point;
      laser_point.x = x + other.x;
      laser_point.y = y + other.y;
      return laser_point;
    };
  };
  double distance_min = 12.0;
  void fuse_pose(const Pose &ndt_estimated, const Pose &scan_odom_motion, const Pose &predict, Pose &fused_pose, Eigen::Matrix3d &fused_cov, const double dt, const PclCloud::Ptr filtered_cloud_ptr, const PclCloud map_cloud, const Pose &scan_trans){
    find_correspondence(filtered_cloud_ptr, map_cloud, predict, scan_trans);
    // Eigen::Matrix3d scan_odom_motion_cov;
    // calculate_motion_covariance(scan_odom_motion, dt, scan_odom_motion_cov);  // オドメトリで得た移動量の共分散
    // Eigen::Matrix3d rotate_scan_odom_motion_cov;
    // rotate_covariance(ndt_estimated, scan_odom_motion_cov, rotate_scan_odom_motion_cov);
    // Eigen::Vector3d mu1(ndt_estimated.x, ndt_estimated.y, ndt_estimated.yaw);
    // Eigen::Vector3d mu2(predict.x, predict.y, predict.yaw);
    // Eigen::Vector3d mu;
    // fuse(mu1, ecov, mu2, mcov, mu, fusedCov);  // 2つの正規分布の融合
  }

  void find_correspondence(const PclCloud::Ptr filtered_cloud_ptr, const PclCloud map_cloud, const Pose &predict, const Pose &scan_trans){
    LaserPoint global;
    LaserPoint *current;
    const LaserPoint *closest_reference;
    vector<const LaserPoint*> current_points;   //対応がとれた現在スキャンの点群
    vector<const LaserPoint*> reference_points;   //対応がとれた参照スキャンの点群

    for (size_t i=0; i<filtered_cloud_ptr->points.size(); i++){
      current->x = filtered_cloud_ptr->points[i].x;
      current->y = filtered_cloud_ptr->points[i].y;
      global.x = filtered_cloud_ptr->points[i].x * scan_trans.yaw + scan_trans.x;
      global.y = filtered_cloud_ptr->points[i].y * scan_trans.yaw + scan_trans.y;
      closest_reference = find_closest_point(map_cloud, global);
    }
    current_points.push_back(current);
    reference_points.push_back(closest_reference);
  }

  const LaserPoint *find_closest_point(PclCloud map_cloud, LaserPoint global){
    LaserPoint cell_min;
    LaserPoint cell_max;
    LaserPoint cell_radius;
    LaserPoint *closest;
    cell_radius.x = cell_radius.y = 0.05;
    cell_min = global - cell_radius;
    cell_max = global + cell_radius;

    for(size_t i=0; i<map_cloud.points.size(); i++){
      if(cell_min.x<map_cloud.points[i].x && cell_max.x>map_cloud.points[i].x && cell_min.y<map_cloud.points[i].y && cell_max.y>map_cloud.points[i].y){
        double distance = (map_cloud.points[i].x - global.x)*(map_cloud.points[i].x - global.x) + (map_cloud.points[i].y - global.y)*(map_cloud.points[i].y - global.y);
        if(distance<distance_min){
          distance_min = distance;
          closest->x = map_cloud.points[i].x;
          closest->y = map_cloud.points[i].y;
        }
      }
    }
    return closest;
  }
};


  // void calculate_motion_covariance(const Pose &scan_odom_motion, const double dt, Eigen::Matrix3d &scan_odom_motion_cov){
  //   double dis = sqrt(scan_odom_motion.x*scan_odom_motion.x + scan_odom_motion.y*scan_odom_motion.y);   // 移動距離
  //   double vt = dis/dt;                    // 並進速度[m/s]
  //   double wt = scan_odom_motion.yaw/dt;     // 角速度[rad/s]
  //   double vthre = 0.02;                   // vtの下限値。同期ずれで0になる場合の対処
  //   double wthre = 0.05;                   // wtの下限値

  //   if (vt < vthre)
  //     vt = vthre;
  //   if (wt < wthre)
  //     wt = wthre;

  //   double dx = vt;
  //   double dy = vt;
  //   double da = wt;

  //   Eigen::Matrix3d C1;
  //   C1.setZero();                          // 対角要素だけ入れる
  //   C1(0,0) = 0.001*dx*dx;                 // 並進成分x
  //   C1(1,1) = 0.005*dy*dy;                 // 並進成分y
  // //  C1(2,2) = 0.005*da*da;                 // 回転成分
  //   C1(2,2) = 0.05*da*da;                 // 回転成分

  //   // スケール調整
  // //  double kk = 100;                     // オドメトリのずれが大きい場合
  //   double kk = 1;                         // 通常
  //   scan_odom_motion_cov = kk*C1;

  //   // 確認用
  //   printf("calMotionCovarianceSimple\n");
  //   printf("vt=%g, wt=%g\n", vt, wt);
  //   double vals[2], vec1[2], vec2[2];
  //   calEigen(scan_odom_motion_cov, vals, vec1, vec2);
  //   printf("scan_odom_motion_cov : %g %g %g %g %g %g\n", scan_odom_motion_cov(0,0), scan_odom_motion_cov(0,1), scan_odom_motion_cov(0,2), scan_odom_motion_cov(1,1), scan_odom_motion_cov(1,2), scan_odom_motion_cov(2,2));
  // }

  // void rotate_covariance(const Pose &ndt_estimated, Eigen::Matrix3d &scan_odom_motion_cov, Eigen::Matrix3d &rotate_scan_odom_motion_cov){
  //   double cs = cos(ndt_estimated.yaw);            // poseの回転成分thによるcos
  //   double sn = sin(ndt_estimated.yaw);
  //   Eigen::Matrix3d J;                            // 回転のヤコビ行列
  //   J << cs, -sn, 0,
  //        sn, cs, 0,
  //        0, 0, 1;

  //   Eigen::Matrix3d JT = J.transpose();
  //   rotate_scan_odom_motion_cov = J*scan_odom_motion_cov*JT;  // 回転変換
  // }
