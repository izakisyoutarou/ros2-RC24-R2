#include "icp_base_slam/icp_base_slam.hpp"

namespace self_localization{
IcpBaseSlam::IcpBaseSlam(const rclcpp::NodeOptions &options) : IcpBaseSlam("", options) {}

IcpBaseSlam::IcpBaseSlam(const std::string& name_space, const rclcpp::NodeOptions &options)
:  rclcpp::Node("icp_base_slam", name_space, options) {
  RCLCPP_INFO(this->get_logger(), "START");
  plot_mode_ = this->get_parameter("plot_mode").as_bool();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  laser_weight_ = this->get_parameter("laser_weight").as_double();
  odom_weight_ = this->get_parameter("odom_weight").as_double();
  trial_num_ = this->get_parameter("trial_num").as_int();
  inlier_dist_threshold_ = this->get_parameter("inlier_dist_threshold").as_double();

  scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "scan", rclcpp::SensorDataQoS(),
  bind(&IcpBaseSlam::scan_callback, this, placeholders::_1));

  odom_linear_subscriber = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
    "can_rx_100",
    _qos,
    std::bind(&IcpBaseSlam::callback_odom_linear, this, std::placeholders::_1)
  );
  odom_angular_subscriber = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
    "can_rx_101",
    _qos,
    std::bind(&IcpBaseSlam::callback_odom_angular, this, std::placeholders::_1)
  );

  ransaced_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "self_localization/ransac",rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  path_publisher = this->create_publisher<nav_msgs::msg::Path>(
    "self_localization/path",rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "self_localization/pose", rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  map_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("self_localization/map", rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());


  ransac_lines = RansacLines(trial_num_, inlier_dist_threshold_);
  create_elephant_map();
  init.x = init_pose_x;
  init.y = init_pose_y;
  odom = init;
  last_scan_odom = init;
  estimated_odom = init;
  last_estimated = init;
}

void IcpBaseSlam::callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  odom_flag=true;
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  double x = (double)bytes_to_float(_candata);
  double y = (double)bytes_to_float(_candata+4);
  diff_odom.x = x - last_odom.x;
  diff_odom.y = y - last_odom.y;
  last_odom.x = x;
  last_odom.y = y;
  odom.x += diff_odom.x;
  odom.y += diff_odom.y;
  estimated_odom.x = odom.x + diff_estimated_sum.x;
  estimated_odom.y = odom.y + diff_estimated_sum.y;
}

void IcpBaseSlam::callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  double yaw = (double)bytes_to_float(_candata);
  diff_odom.yaw = yaw - last_odom.yaw;
  odom.yaw += diff_odom.yaw;
  last_odom.yaw = odom.yaw;
  estimated_odom.yaw = normalize_yaw(odom.yaw + diff_estimated_sum.yaw);
}

void IcpBaseSlam::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  scan_execution_time_start = chrono::system_clock::now();
  double current_scan_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_scan = current_scan_received_time - last_scan_received_time;
  last_scan_received_time = current_scan_received_time;
  if (dt_scan > 0.03 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "scan time interval is too large->%f", dt_scan);
  }
  current_scan_odom = estimated_odom;
  double odom_to_lidar_x = odom_to_lidar_length * cos(current_scan_odom.yaw);
  double odom_to_lidar_y = odom_to_lidar_length * sin(current_scan_odom.yaw);
  vector<config::LaserPoint> src_points = converter.scan_to_vector(msg, current_scan_odom, odom_to_lidar_x, odom_to_lidar_y);

  ransac_lines.fuse_inliers(src_points, current_scan_odom, odom_to_lidar_x, odom_to_lidar_y);
  vector<config::LaserPoint> line_points = ransac_lines.get_sum();
  diff_estimated = ransac_lines.get_estimated_diff();
  if(odom_flag){
    diff_estimated_sum += diff_estimated;
    odom_flag=false;
  }

  pointcloud2_view(line_points);
  scan_execution_time_end = chrono::system_clock::now();
  scan_execution_time = chrono::duration_cast<chrono::milliseconds>(scan_execution_time_end-scan_execution_time_start).count();
  // RCLCPP_INFO(this->get_logger(), "scan execution time->%d", scan_execution_time);
}

void IcpBaseSlam::pointcloud2_view(vector<config::LaserPoint> &points){
  sensor_msgs::msg::PointCloud2 cloud = converter.vector_to_PC2(points);

  // Eigen::Quaterniond quat_eig =
  //   Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
  //   Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
  //   Eigen::AngleAxisd(estimated_odom.yaw, Eigen::Vector3d::UnitZ());

  // geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
  // corrent_pose_stamped.header.stamp = this->now();
  // corrent_pose_stamped.header.frame_id = "map";
  // corrent_pose_stamped.pose.position.x = estimated_odom.x;
  // corrent_pose_stamped.pose.position.y = estimated_odom.y;
  // corrent_pose_stamped.pose.position.z = 0.0;
  // corrent_pose_stamped.pose.orientation = quat_msg;

  // path.header.stamp = this->now();
  // path.header.frame_id = "map";
  // path.poses.push_back(corrent_pose_stamped);


  map_publisher->publish(map_cloud);
  ransaced_publisher->publish(cloud);
  // pose_publisher->publish(corrent_pose_stamped);
  // path_publisher->publish(path);
}


void IcpBaseSlam::create_elephant_map(){
  config::LaserPoint map_point;
  //右の垂木
  for(int i=0; i<=int((map_point_x[2] - map_point_x[0])*1000); i++){
    map_point.x = double(i)/1000 + map_point_x[0];
    map_point.y = map_point_y[0];
    map_points.push_back(map_point);
  }
  //後ろの垂木
  for(int i=0; i<=int((map_point_y[3] - map_point_y[0])*1000); i++){
    map_point.x = map_point_x[0];
    map_point.y = double(i)/1000 + map_point_y[0];
    map_points.push_back(map_point);
  }
  //左の垂木
  for(int i=0; i<=int((map_point_x[2] - map_point_x[0])*1000); i++){
    map_point.x = double(i)/1000 + map_point_x[0];
    map_point.y = map_point_y[3];
    map_points.push_back(map_point);
  }
  //右奥正面向きのフェンス
  for(int i=0; i<=int((map_point_y[1] - map_point_y[0])*1000); i++){
    map_point.x = map_point_x[2];
    map_point.y = double(i)/1000 + map_point_y[0];
    map_points.push_back(map_point);
  }
  //右縦向きのフェンス
  for(int i=0; i<=int((map_point_x[2] - map_point_x[1])*1000); i++){
    map_point.x = double(i)/1000 + map_point_x[1];
    map_point.y = map_point_y[1];
    map_points.push_back(map_point);
  }
  //正面のフェンス
  for(int i=0; i<=int((map_point_y[2] - map_point_y[1])*1000); i++){
    map_point.x = map_point_x[1];
    map_point.y = double(i)/1000 + map_point_y[1];
    map_points.push_back(map_point);
  }
  //左縦向きのフェンス
  for(int i=0; i<=int((map_point_x[2] - map_point_x[1])*1000); i++){
    map_point.x = double(i)/1000 + map_point_x[1];
    map_point.y = map_point_y[2];
    map_points.push_back(map_point);
  }
  for(int i=0; i<=int((map_point_y[3] - map_point_y[2])*1000); i++){
    map_point.x = map_point_x[2];
    map_point.y = double(i)/1000 + map_point_y[2];
    map_points.push_back(map_point);
  }
  map_cloud = converter.vector_to_PC2(map_points);
}
}
