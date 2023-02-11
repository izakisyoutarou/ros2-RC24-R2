#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "config.hpp"

using namespace std;
using PointType = pcl::PointXYZ;
using PclCloud = pcl::PointCloud<PointType>;

class RansacLines{
public:
  RansacLines(){};
  RansacLines(int &trial_num, double &inlier_dist_threshold) : trial_num_(trial_num), inlier_dist_threshold_(inlier_dist_threshold){};
  ~RansacLines(){};
  void fuse_inliers(vector<config::LaserPoint> &src_points, const Pose &estimated, double &odom_to_lidar_x, double &odom_to_lidar_y);
  void init();
  EstimatedLine calc_inliers(vector<config::LaserPoint> &divided_points);
  void get_inliers();
  void devide_points(vector<config::LaserPoint> &src_points);
  void set_points(vector<config::LaserPoint> &points, double map_point_x_1, double map_point_x_2, double map_point_y_1, double map_point_y_2, config::LaserPoint &src_point);
  void input_points(EstimatedLine &line);
  bool clear_points(EstimatedLine &estimated_line, int size_threshold, int angle_threshold_min, int angle_threshold_max);
  double calc_diff_angle();
  double LPF(double raw);
  void calc_estimated_diff(const Pose &estimated, double &odom_to_lidar_x, double &odom_to_lidar_y);
  EstimatedLine get_filtered_rafter_right();
  EstimatedLine get_filtered_rafter_left();
  EstimatedLine get_filtered_rafter_back();
  EstimatedLine get_filtered_fence_centor_right();
  EstimatedLine get_filtered_fence_centor_left();
  EstimatedLine get_filtered_fence_front();
  EstimatedLine get_filtered_fence_right();
  EstimatedLine get_filtered_fence_left();
  vector<config::LaserPoint> get_sum();
  Pose get_estimated_diff();

private:
  vector<vector<config::LaserPoint>> lines;
  /* linesの順番
  0:rafter_right
  1:fence_right
  2:fence_left
  3:rafter_left
  4:rafter_back
  5:fence_front
  6:fence_centor_right
  7:fence_centor_left
  */
  vector<EstimatedLine> lines_;

  vector<config::LaserPoint> sum;

  EstimatedLine inlier;

  int trial_num_;
  double inlier_dist_threshold_;
  double detect_length = 0.0;
  double distance_threshold=0.8;
  double last_lpf=0;
  chrono::system_clock::time_point time_start, time_end;
  Pose estimated_diff;
};
