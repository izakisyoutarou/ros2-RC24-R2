
#pragma once

#include <iostream>

#include "sensor_msgs/msg/laser_scan.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_eigen/tf2_eigen.h>

#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "utilities/can_utils.hpp"

#include "icp_base_slam/config.hpp"
#include "icp_base_slam/pose_fuser.hpp"
#include "icp_base_slam/ransac_lines.hpp"
#include "icp_base_slam/dynamic_voxel_grid_filter.hpp"
#include "icp_base_slam/vector_to_PC2.hpp"

#include "visibility.h"

using namespace std;

namespace self_localization{
class IcpBaseSlam : public rclcpp::Node{
public:
  ICP_BASE_SLAM_PUBLIC
  explicit IcpBaseSlam(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ICP_BASE_SLAM_PUBLIC
  explicit IcpBaseSlam(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PoseFuser *pose_fuser;  // センサ融合器
  RansacLines ransac_lines;
  DynamicVoxelGridFilter dynamic_voxel_grid_filter;
  VectorToPC2 vector_to_PC2;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void create_elephant_map();
  void pointcloud2_view(vector<config::LaserPoint> &points);
  // pcl::PointCloud<pcl::PointXYZRGB> cloud_add_collor(PclCloud &cloud, char *rgb);
  double normalize_yaw(double yaw){
    if (yaw < -M_PI) yaw += 2*M_PI;
    else if (yaw >= M_PI) yaw -= 2*M_PI;
    return yaw;
  }

  sensor_msgs::msg::PointCloud2 map_cloud;
  vector<config::LaserPoint> map_points;

  chrono::system_clock::time_point time_start, time_end, scan_execution_time_start, scan_execution_time_end, align_time_start, align_time_end, fuse_time_start, fuse_time_end;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_linear_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_angular_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ransaced_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
  rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped corrent_pose_stamped;

  double init_pose_x = -5.45;
  double init_pose_y = 0.0;
  double last_scan_received_time=0.0;
  double last_odom_received_time=0.0;
  int scan_execution_time=0;

  Pose init;
  Pose odom;
  Pose current_scan_odom;
  Pose estimated_odom;
  Pose last_odom;
  Pose diff_odom;
  Pose pose;
  Pose last_estimated;
  Pose ndt_estimated;
  Pose last_scan_odom;
  Pose estimated;
  Pose diff_estimated;
  Pose diff_estimated_sum;
  double odom_to_lidar_length = 0.4655;
  double reso = 0.125;
  double view_ranges = 270.0;
  bool odom_flag=false;

  ///////////////////////////////////////////////チューニング///////////////////////////////////////////////
  bool plot_mode_;
  double voxel_leaf_size_; //ダウンサンプリングボクセル
  double laser_weight_;
  double odom_weight_;
  int trial_num_;
  double inlier_dist_threshold_;
};
}
