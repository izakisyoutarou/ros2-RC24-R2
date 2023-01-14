
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

#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "utilities/can_utils.hpp"

#include "config.hpp"
#include "icp_base_slam/pose_fuser.hpp"
#include "icp_base_slam/ransac_lines.hpp"
#include "icp_base_slam/dynamic_voxel_grid_filter.hpp"

#include "visibility.h"

using PointType = pcl::PointXYZ;
using PclCloud = pcl::PointCloud<PointType>;

namespace self_localization{
class IcpBaseSlam : public rclcpp::Node{
public:
  ICP_BASE_SLAM_PUBLIC
  explicit IcpBaseSlam(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ICP_BASE_SLAM_PUBLIC
  explicit IcpBaseSlam(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  double degToRad(double degree){return degree*M_PI/180;}
  double radToDeg(double rad){return rad*180/M_PI;}

private:
  PoseFuser *pose_fuser;  // センサ融合器
  RansacLines *ransac_lines;
  DynamicVoxelGridFilter dynamic_voxel_grid_filter;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void simulator_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

  void create_elephant_map();
  void pointcloud2_view(PclCloud &map_cloud, PclCloud &ransac_cloud);
  void path_view(const Pose &estimate_point, const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
  void path_view_from_simulator(const Pose &estimate_point, const nav_msgs::msg::Odometry::SharedPtr msg);
  pcl::PointCloud<pcl::PointXYZRGB> cloud_add_collor(PclCloud &cloud, char *rgb);

  double quaternionToYaw(double x, double y, double z, double w);
  int max_time(int num);
  int max_iteration(int num);
  double normalize_yaw(double yaw){
    if (yaw < -M_PI) yaw += 2*M_PI;
    else if (yaw >= M_PI) yaw -= 2*M_PI;
    return yaw;
  }

  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
  pcl::NormalDistributionsTransform2D<PointType, PointType> ndt2d;
  pcl::Registration<PointType, PointType>::Ptr registration_;

  PclCloud cloud;
  PclCloud input_circle_cloud;
  PclCloud input_elephant_cloud;
  PclCloud global_cloud;
  PclCloud inlier_cloud;
  pcl::VoxelGrid<PointType> voxel_grid_filter;
  pcl::VoxelGrid<PointType> map_voxel_grid_filter;

  chrono::system_clock::time_point time_start, time_end, scan_execution_time_start, scan_execution_time_end, align_time_start, align_time_end, fuse_time_start, fuse_time_end;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr simulator_odom_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_linear_subscriber;
  rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr odom_angular_subscriber;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pointcloud2_publisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ransac_pointcloud2_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher;
  rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped corrent_pose_stamped;

  int last_num_time=0;
  int last_num_iteration=0;
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
  double reso = 0.125;
  double view_ranges = 270.0;

  //マップモデルのパラメータ
  double rafter_width = 0.05;

  ///////////////////////////////////////////////チューニング///////////////////////////////////////////////
  std::string registration_method_;
  std::string filtering_method_;
  double voxel_leaf_size_; //ダウンサンプリングボクセル

  double laser_weight_;
  double odom_weight_;

  int trial_num_;
  double inlier_dist_threshold_;

  bool plot_mode_{true};
  bool use_gazebo_simulator_{true};
};
}
