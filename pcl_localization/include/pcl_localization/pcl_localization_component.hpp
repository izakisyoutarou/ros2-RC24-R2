#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <pcl/registration/ndt.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "pcl_localization/lidar_undistortion.hpp"

#include "my_messages/msg/pose.hpp"
#include "my_messages/msg/odom_delay.hpp"
#define ONE_GIGA   1000000000

using namespace std;
using namespace std::chrono_literals;
using PointType = pcl::PointXYZ;

class PCLLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PCLLocalization(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & state);

  void initializeParameters();
  void initializePubSub();
  void initializeRegistration();
  void initialPoseReceived(geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void odomReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
  void imuReceived(sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void scanReceived(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void create_elephant_map();
  void odom_delay_callback(const my_messages::msg::OdomDelay::SharedPtr msg);
  double radToDeg(double rad){return rad*180/M_PI;}
  void cloud_view(pcl::PointCloud<PointType> map_cloud, pcl::PointCloud<PointType> input_cloud);
  double quaternionToYaw(double x, double y, double z, double w);
  tf2_ros::TransformBroadcaster broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::ConstSharedPtr initial_pose_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub_;
  rclcpp::Subscription<my_messages::msg::OdomDelay>::SharedPtr odom_delay_subscriber;

  pcl::VoxelGrid<PointType> voxel_grid_filter_;
  geometry_msgs::msg::PoseStamped corrent_pose_stamped_;
  nav_msgs::msg::Path path_;
  pcl::PointCloud<PointType> input_elephant_cloud;
  pcl::PointCloud<PointType> cloud;

  bool map_recieved_{false};
  bool initialpose_recieved_{false};

  // parameters
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  double scan_period_;
  double ndt_resolution_;
  double ndt_step_size_;
  double transform_epsilon_;
  double voxel_leaf_size_;
  bool use_pcd_map_{false};
  bool set_initial_pose_{false};
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_qx_;
  double initial_pose_qy_;
  double initial_pose_qz_;
  double initial_pose_qw_;

  bool use_odom_{false};
  double last_odom_received_time_ = 0.0;
  bool use_imu_{false};
  bool enable_debug_{false};

  // imu
  LidarUndistortion lidar_undistortion_;

  chrono::system_clock::time_point time_start, time_end, receive_time, dt_odom; // 型は auto で可
  int last_receive_time=0;
  int last_pose_x = 0;
  int last_pose_y = 0;
  int last_pose_yaw = 0;
  pcl::NormalDistributionsTransform<PointType, PointType> ndt;
};
