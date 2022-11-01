
#pragma once

#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include "sensor_msgs/msg/laser_scan.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "my_messages/msg/odom_delay.hpp"

#include "config.hpp"
#include "icp_base_slam/pose_fuser.hpp"

#include "visibility.h"

class IcpBaseSlam : public rclcpp::Node{
public:
  ICP_BASE_SLAM_PUBLIC
  explicit IcpBaseSlam(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ICP_BASE_SLAM_PUBLIC
  explicit IcpBaseSlam(const std::string& node_name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void set_odom(double x, double y, double yaw);
  void update_data(double trans_pose_x, double trans_pose_y, double trans_pose_yaw);
  double degToRad(double degree){return degree*M_PI/180;}
  double radToDeg(double rad){return rad*180/M_PI;}

private:
  void make_input_circles();
  double circle_model_x(int i, int model_count);
  double circle_model_y_inc(double point_x, double circle_y);
  double circle_model_y_dec(double point_x, double circle_y);
  void create_elephant_map();
  void print4x4Matrix (const Eigen::Matrix4d & matrix);
  void input_cloud_view(PclCloud input_cloud);
  void icp_cloud_view(PclCloud map_cloud, PclCloud input_cloud);
  void pointcloud2_view(PclCloud::Ptr cloud_ptr, PclCloud map_cloud);

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_delay_callback(const my_messages::msg::OdomDelay::SharedPtr msg);
  void simulator_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double quaternionToYaw(double x, double y, double z, double w);
  int max_time(int num);
  int max_iteration(int num);

  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  PclCloud cloud;
  PclCloud input_circle_cloud;
  PclCloud input_elephant_cloud;
  PclCloud global_cloud;
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  pcl::VoxelGrid<PointType> map_voxel_grid_filter;

  chrono::system_clock::time_point  time_start, time_end; // 型は auto で可

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<my_messages::msg::OdomDelay>::SharedPtr odom_delay_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr simulator_odom_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pointcloud2_publisher;


  int last_num_time=0;
  int last_num_iteration=0;
  double init_pose_x = -5.5;
  double init_pose_y = 0.0;

  Pose odom;
  Pose last_odom;
  Pose pose;
  Pose last_estimated;
  Pose estimated;
  Pose ndt_estimated;
  Pose current_scan_odom;
  Pose last_scan_odom;
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
  //ndt ndt_resolutionとvoxel_leaf_sizeは密接な関係あり
  double transformation_epsilon = 0.005;//最新の変換とそのひとつ前の変換の差の閾値。
  double ndt_resolution = 0.005;  //ndtボクセルサイズ
  double ndt_step_size = 0.00005;  //探索領域を区切るサイズ(小さいと解の精度が上がり、処理が重くなる)
  double voxel_leaf_size = 0.5; //ダウンサンプリングボクセル
  double map_voxel_leaf_size = 0.01;

  bool use_odom{false};
  bool use_gazebo_simulator{true};
  PoseFuser *pose_fuser;  // センサ融合器
};
