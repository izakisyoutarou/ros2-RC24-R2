
#pragma once

#include <iostream>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "controller_interface_msg/msg/base_control.hpp"

#include "utilities/can_utils.hpp"

#include "RANSAC_localization/config.hpp"
#include "RANSAC_localization/pose_fuser.hpp"
#include "RANSAC_localization/detect_lines.hpp"
#include "RANSAC_localization/converter.hpp"
#include "RANSAC_localization/voxel_grid_filter.hpp"

#include "visibility.h"

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

namespace self_localization{
class RANSACLocalization : public rclcpp::Node{
public:
  RANSAC_LOCALIZATION_PUBLIC
  explicit RANSACLocalization(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  RANSAC_LOCALIZATION_PUBLIC
  explicit RANSACLocalization(const string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PoseFuser pose_fuser;  // センサ融合器
  DtectLines detect_lines;
  Converter converter;
  VoxelGridFilter voxel_grid_filter;
  void callback_restart(const controller_interface_msg::msg::BaseControl::SharedPtr msg);
  void callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void init();
  void create_elephant_map();
  void publishers(vector<LaserPoint> &points);
  LaserPoint rotate(LaserPoint point, double theta);
  vector<LaserPoint> transform(const vector<LaserPoint> &points, const Vector3d &pose);
  Vector3d calc_body_to_sensor(const Vector6d& sensor_pos);

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_linear_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_angular_subscriber;
  rclcpp::Subscription<controller_interface_msg::msg::BaseControl>::SharedPtr restart_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ransaced_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr self_pose_publisher;
  geometry_msgs::msg::Vector3 vector_msg;
  sensor_msgs::msg::PointCloud2 map_cloud;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped corrent_pose_stamped;
  rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

  vector<LaserPoint> map_points;
  Vector3d init_pose = Vector3d::Zero();
  Vector3d odom = Vector3d::Zero();
  Vector3d last_odom = Vector3d::Zero();
  Vector3d est_diff_sum = Vector3d::Zero();
  Vector3d last_estimated = Vector3d::Zero();
  Vector6d tf_laser2robot = Vector6d::Zero();
  double last_odom_received_time=0.0;
  double last_jy_received_time=0.0;
  double last_scan_received_time=0.0;

  chrono::system_clock::time_point time_start, time_end;

  bool plot_mode_;
};
}
