
#pragma once

#include <iostream>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"

#include "utilities/can_utils.hpp"

#include "config.hpp"
#include "pose_fuser.hpp"
#include "detect_lines.hpp"
#include "converter.hpp"
#include "voxel_grid_filter.hpp"

#include "visibility.h"

using namespace std;
using namespace Eigen;

namespace self_localization{
class ransaclocalization : public rclcpp::Node{
public:
  RANSAC_LOCALIZATION_PUBLIC
  explicit ransaclocalization(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  RANSAC_LOCALIZATION_PUBLIC
  explicit ransaclocalization(const string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PoseFuser pose_fuser;  // センサ融合器
  DetectLines detect_lines;
  Converter converter;
  VoxelGridFilter voxel_grid_filter;
  void callback_restart(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
  void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void load_map_config(vector<LineData> &lines);
  void init();
  void create_map(vector<LineData> &lines);
  void publishers(vector<LaserPoint> &points);
  void update(const Vector3d &estimated, const Vector3d &current_scan_odom);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_linear_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_angular_subscriber;
  rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr restart_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ransaced_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr self_pose_publisher;
  rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr init_angle_publisher;

  geometry_msgs::msg::Vector3 vector_msg;
  geometry_msgs::msg::Vector3 odom_msg;
  sensor_msgs::msg::PointCloud2 map_cloud;
  geometry_msgs::msg::PoseStamped corrent_pose_stamped;
  geometry_msgs::msg::PoseStamped odom_stamped;
  rclcpp::QoS _qos = rclcpp::QoS(rclcpp::KeepLast(10));
  rclcpp::QoS fast_qos = rclcpp::QoS(rclcpp::KeepLast(1));

  vector<LaserPoint> map_points;
  Vector3d init_pose = Vector3d::Zero();
  Vector3d odom = Vector3d::Zero();
  Vector3d last_odom = Vector3d::Zero();
  Vector3d diff_odom = Vector3d::Zero();
  Vector3d est_diff_sum = Vector3d::Zero();
  Vector3d last_estimated = Vector3d::Zero();
  Vector6d tf_laser2robot = Vector6d::Zero();


  double last_odom_received_time=0.0;
  double last_jy_received_time=0.0;
  double last_scan_received_time=0.0;
  double dt_scan=0.0;
  double linear_vel=0.0;
  double angular_vel=0.0;

  bool init_flag{false};
  bool init_jy_time_flag{true};

  Vector3d correction_rate_ave = Vector3d::Zero();
  Vector3d correction_rate_sum = Vector3d::Zero();
  Vector3d trans = Vector3d::Zero();
  Vector3i correction_count = Vector3i::Zero();

  chrono::system_clock::time_point time_start, time_end;

  int count=0;

  vector<double> initial_pose_;
  vector<double> second_initial_pose_;
  vector<double> tf_array;

  const bool plot_mode_;
  const double laser_weight_;
  const double odom_weight_liner_;
  const double odom_weight_angler_;
  const double voxel_size_;
  const int trial_num_;
  const double inlier_dist_threshold_;
  const double inlier_length_threshold_;
  const int odom_linear_can_id;
  const int odom_angular_can_id;
  const int init_angular_can_id;
};
}