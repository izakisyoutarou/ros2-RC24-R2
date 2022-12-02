
#pragma once

#include <iostream>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/ndt_2d.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2_eigen/tf2_eigen.h>

#include "config.hpp"
#include "icp_base_slam/pose_fuser.hpp"

#include "visibility.h"

using PointType = pcl::PointXYZ;
using PclCloud = pcl::PointCloud<PointType>;
namespace self_localization{
class IcpBaseSlam : public rclcpp::Node{
public:
  ICP_BASE_SLAM_PUBLIC
  explicit IcpBaseSlam(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  // explicit IcpBaseSlam(const rclcpp::NodeOptions& options);
  ICP_BASE_SLAM_PUBLIC
  explicit IcpBaseSlam(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  double degToRad(double degree){return degree*M_PI/180;}
  double radToDeg(double rad){return rad*180/M_PI;}

private:
  PoseFuser *pose_fuser;  // センサ融合器

  void make_input_circles();
  double circle_model_x(int i, int model_count);
  double circle_model_y_inc(double point_x, double circle_y);
  double circle_model_y_dec(double point_x, double circle_y);
  void create_elephant_map();
  void print4x4Matrix (const Eigen::Matrix4d & matrix);
  void pointcloud2_view(PclCloud::Ptr cloud_ptr, PclCloud map_cloud, const Pose estimated);
  void path_view(const Pose &estimate_point, const nav_msgs::msg::Odometry::SharedPtr msg);

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void simulator_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double quaternionToYaw(double x, double y, double z, double w);
  int max_time(int num);
  int max_iteration(int num);

  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  pcl::NormalDistributionsTransform2D<PointType, PointType> ndt2d;
  pcl::Registration<PointType, PointType>::Ptr registration_;

  PclCloud cloud;
  PclCloud input_circle_cloud;
  PclCloud input_elephant_cloud;
  PclCloud global_cloud;
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  pcl::VoxelGrid<PointType> map_voxel_grid_filter;

  chrono::system_clock::time_point time_start, time_end, scan_execution_time_start, scan_execution_time_end, align_time_start, align_time_end, fuse_time_start, fuse_time_end;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr simulator_odom_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pointcloud2_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped corrent_pose_stamped;

  int last_num_time=0;
  int last_num_iteration=0;
  double init_pose_x = -5.5;
  double init_pose_y = 0.0;
  double last_scan_received_time=0.0;
  double last_odom_received_time=0.0;
  int scan_execution_time=0;

  Pose init;
  Pose odom;
  Pose estimated_odom;
  Pose last_odom;
  Pose diff_odom;
  Pose pose;
  Pose last_estimated;
  Pose ndt_estimated;
  Pose last_scan_odom;
  Pose estimated;
  Pose diff_estimated;
  double reso = 0.125;
  double view_ranges = 270.0;

  //マップモデルのパラメータ
  double circle_x = 2.8;
  double circle_right_y = 2.8;
  double circle_center_y = 6.0;
  double circle_left_y = 9.2;
  double R=0.05;
  double rafter_width = 0.05;
  int model_count=0;

  ///////////////////////////////////////////////チューニング///////////////////////////////////////////////
  std::string registration_method_;
  //ndt ndt_resolutionとvoxel_leaf_sizeは密接な関係あり
  double transformation_epsilon_;//最新の変換とそのひとつ前の変換の差の閾値。
  double ndt_resolution_;  //ndtボクセルサイズ
  double ndt_step_size_;  //探索領域を区切るサイズ
  double voxel_leaf_size_; //ダウンサンプリングボクセル

  int maximum_iterations_;
  double grid_centre_x_;
  double grid_centre_y_;
  double grid_extent_x_;
  double grid_extent_y_;
  double grid_step_;
  double optimization_step_size_;
  double transformation_epsilon_2d_;
  double map_voxel_leaf_size_;

  double laser_weight_;
  double odom_weight_;

  bool plot_mode_{true};
  bool use_gazebo_simulator_{true};
};
}
