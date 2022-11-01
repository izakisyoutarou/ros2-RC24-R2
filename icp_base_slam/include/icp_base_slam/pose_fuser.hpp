#pragma once
#include "config.hpp"
#include "icp_base_slam/icp_base_slam.hpp"
#include <vector>
//lidarとオドメトリの推定値を融合する。(imuの融合器はその後作る)
class PoseFuser{
private:
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
    }
  };

public:
  PoseFuser(){}

  const LaserPoint *get_data(const LaserPoint *data){
    return data;
  }

  void fuse_pose(const Pose &ndt_estimated, const Pose &scan_odom_motion, const Pose &predict, Pose &fused_pose, Eigen::Matrix3d &fused_cov, const double dt, const PclCloud global_cloud, const PclCloud map_cloud, const Pose &scan_trans){
    find_correspondence(global_cloud, map_cloud, predict, scan_trans);
    // Eigen::Matrix3d scan_odom_motion_cov;
    // calculate_motion_covariance(scan_odom_motion, dt, scan_odom_motion_cov);  // オドメトリで得た移動量の共分散
    // Eigen::Matrix3d rotate_scan_odom_motion_cov;
    // rotate_covariance(ndt_estimated, scan_odom_motion_cov, rotate_scan_odom_motion_cov);
    // Eigen::Vector3d mu1(ndt_estimated.x, ndt_estimated.y, ndt_estimated.yaw);
    // Eigen::Vector3d mu2(predict.x, predict.y, predict.yaw);
    // Eigen::Vector3d mu;
    // fuse(mu1, ecov, mu2, mcov, mu, fusedCov);  // 2つの正規分布の融合
  }

  void find_correspondence(const PclCloud global_cloud, const PclCloud map_cloud, const Pose &predict, const Pose &scan_trans){
    vector<LaserPoint> current_points;   //対応がとれた現在スキャンの点群
    vector<LaserPoint> reference_points;   //対応がとれた参照スキャンの点群
    current_points.clear();
    reference_points.clear();
    LaserPoint global;
    LaserPoint closest_reference;

    for (size_t i=0; i<global_cloud.points.size(); i++){
      global.x = global_cloud.points[i].x;
      global.y = global_cloud.points[i].y;
      closest_reference = find_closest_vertical_point(global);
    }
    current_points.push_back(global);
    reference_points.push_back(closest_reference);
  }

  LaserPoint find_closest_vertical_point(LaserPoint global){
    LaserPoint closest;
    LaserPoint vertical_distance;
    double distance_min_x = 100.0;
    double distance_min_y = 100.0;
    double back_rafter_x = 0.05-6.0;
    double fence_x = 2.0-6.0;
    double front_rafter_x = 5.9875-6.0;
    double right_rafter_y = 0.05-6.0;
    double right_fence_y = 2.0-6.0;
    double left_fence_y = 10.0-6.0;
    double left_rafter_y = 11.95-6.0;
    double x[4] = {back_rafter_x, fence_x, front_rafter_x, 100.0};  //4つ目が最小距離にならないようにしている。
    double y[4] = {right_rafter_y, right_fence_y, left_fence_y, left_rafter_y};
    for(int i=0; i<4; i++){
      vertical_distance.x = fabs(x[i] - global.x);
      vertical_distance.y = fabs(y[i] - global.y);
      if(vertical_distance.x < distance_min_x){
        distance_min_x = vertical_distance.x;
        closest.x = x[i];
      }
      if(vertical_distance.y < distance_min_y){
        distance_min_y = vertical_distance.y;
        // printf("distance_min_y -> %f\n", distance_min_y);
        closest.y = y[i];
      }
    }
    return closest;
  }

  LaserPoint find_closest_point_in_voxel(PclCloud map_cloud, LaserPoint global){
    double distance_min = 12.0;
    LaserPoint cell_min;
    LaserPoint cell_max;
    LaserPoint cell_radius;
    LaserPoint closest;
    cell_radius.x = cell_radius.y = 0.05;
    cell_min = global - cell_radius;
    cell_max = global + cell_radius;
    for(size_t i=0; i<map_cloud.points.size(); i++){
      if(cell_min.x<map_cloud.points[i].x && cell_max.x>map_cloud.points[i].x && cell_min.y<map_cloud.points[i].y && cell_max.y>map_cloud.points[i].y){
        double distance = (map_cloud.points[i].x - global.x)*(map_cloud.points[i].x - global.x) + (map_cloud.points[i].y - global.y)*(map_cloud.points[i].y - global.y);
        if(distance<distance_min){
          distance_min = distance;
          closest.x = map_cloud.points[i].x;
          closest.y = map_cloud.points[i].y;
        }
      }
    }
    return closest;
  }
};

// void calculate_ndt_covariance(const Pose &pose, vector<const LaserPoint*> &current_points, vector<const LaserPoint*> &reference_points, Eigen::Matrix3d &covariance){

// }


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
