#include "icp_base_slam/icp_base_slam.hpp"

namespace self_localization{
IcpBaseSlam::IcpBaseSlam(const rclcpp::NodeOptions &options) : IcpBaseSlam("", options) {}

IcpBaseSlam::IcpBaseSlam(const string& name_space, const rclcpp::NodeOptions &options)
:  rclcpp::Node("icp_base_slam", name_space, options) {
  RCLCPP_INFO(this->get_logger(), "START");
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
    bind(&IcpBaseSlam::callback_odom_linear, this, placeholders::_1)
  );
  odom_angular_subscriber = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
    "can_rx_101",
    _qos,
    bind(&IcpBaseSlam::callback_odom_angular, this, placeholders::_1)
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
  last_estimated = init;
  raw = init;
}

void IcpBaseSlam::callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  odom_flag=true;
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  double x = (double)bytes_to_float(_candata);
  double y = (double)bytes_to_float(_candata+4);
  diff_odom.x = x - last_odom.x;
  diff_odom.y = y - last_odom.y;
  odom.x += diff_odom.x;
  odom.y += diff_odom.y;
  raw.x += diff_odom.x;
  raw.y += diff_odom.y;
  last_odom.x = x;
  last_odom.y = y;
}

void IcpBaseSlam::callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  double yaw = (double)bytes_to_float(_candata);
  diff_odom.yaw = yaw - last_odom.yaw;
  odom.yaw += diff_odom.yaw;
  odom.yaw = normalize_yaw(odom.yaw);
  last_odom.yaw = odom.yaw;

}

void IcpBaseSlam::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  scan_execution_time_start = chrono::system_clock::now();

  double current_scan_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_scan = current_scan_received_time - last_scan_received_time;
  last_scan_received_time = current_scan_received_time;
  if (dt_scan > 0.03 /* [sec] */) RCLCPP_WARN(this->get_logger(), "scan time interval is too large->%f", dt_scan);

  current_scan_odom = odom;
  Pose scan_odom_motion = current_scan_odom - last_estimated; //前回scanからのオドメトリ移動量
  // RCLCPP_INFO(this->get_logger(), "current_scan_odom.x    >%f", current_scan_odom.x);
  // RCLCPP_INFO(this->get_logger(), "last_estimated.x    >%f", last_estimated.x);

  double odom_to_lidar_x = odom_to_lidar_length * cos(current_scan_odom.yaw);
  double odom_to_lidar_y = odom_to_lidar_length * sin(current_scan_odom.yaw);
  vector<config::LaserPoint> src_points = converter.scan_to_vector(msg, current_scan_odom, odom_to_lidar_x, odom_to_lidar_y);

  ransac_lines.fuse_inliers(src_points, current_scan_odom, odom_to_lidar_x, odom_to_lidar_y);
  vector<config::LaserPoint> line_points = ransac_lines.get_sum();
  Pose trans = ransac_lines.get_estimated_diff();
  // RCLCPP_INFO(this->get_logger(), "trans.x>%f", trans.x);
  // if(odom_flag){
  //   odom_flag=false;
  // }
  ransac_estimated = current_scan_odom + trans;
  vector<config::LaserPoint> global_points = transform(line_points, trans);
  // vector<config::LaserPoint> global_points = converter.scan_to_vector(msg, ransac_estimated, odom_to_lidar_x, odom_to_lidar_y);

  Pose estimated = pose_fuser.fuse_pose(ransac_estimated, scan_odom_motion, current_scan_odom, dt_scan, src_points, global_points, laser_weight_, odom_weight_);
  // RCLCPP_INFO(this->get_logger(), "estimated.x>%f", estimated.x);

  Pose diff_estimated = estimated - current_scan_odom;
  // RCLCPP_INFO(this->get_logger(), "diff_estimated.x>%f", diff_estimated.x);
  odom += diff_estimated;
  last_estimated = estimated;

  pointcloud2_view(line_points);
  scan_execution_time_end = chrono::system_clock::now();
  scan_execution_time = chrono::duration_cast<chrono::milliseconds>(scan_execution_time_end-scan_execution_time_start).count();
  // RCLCPP_INFO(this->get_logger(), "scan execution time->%d", scan_execution_time);
  // RCLCPP_INFO(this->get_logger(), "estimated.y>%f", estimated.y);
  // RCLCPP_INFO(this->get_logger(), "estimated.yaw>%f°", radToDeg(estimated.yaw));
}

void IcpBaseSlam::pointcloud2_view(vector<config::LaserPoint> &points){
  sensor_msgs::msg::PointCloud2 cloud = converter.vector_to_PC2(points);

  corrent_pose_stamped.header.stamp = this->now();
  corrent_pose_stamped.header.frame_id = "map";
  corrent_pose_stamped.pose.position.x = odom.x;
  corrent_pose_stamped.pose.position.y = odom.y;
  corrent_pose_stamped.pose.orientation.z = sin(odom.yaw / 2.0);
  corrent_pose_stamped.pose.orientation.w = cos(odom.yaw / 2.0);

  // path.header.stamp = this->now();
  // path.header.frame_id = "map";
  // path.poses.push_back(corrent_pose_stamped);


  map_publisher->publish(map_cloud);
  ransaced_publisher->publish(cloud);
  pose_publisher->publish(corrent_pose_stamped);
  // path_publisher->publish(path);
}


void IcpBaseSlam::create_elephant_map(){
  config::LaserPoint map_point;
  //右の垂木
  for(int i=0; i<=int((map_point_x[2] - map_point_x[0])*1000); i++){
    map_point.x = static_cast<double>(i)/1000 + map_point_x[0];
    map_point.y = map_point_y[0];
    map_points.push_back(map_point);
  }
  //後ろの垂木
  for(int i=0; i<=int((map_point_y[3] - map_point_y[0])*1000); i++){
    map_point.x = map_point_x[0];
    map_point.y = static_cast<double>(i)/1000 + map_point_y[0];
    map_points.push_back(map_point);
  }
  //左の垂木
  for(int i=0; i<=int((map_point_x[2] - map_point_x[0])*1000); i++){
    map_point.x = static_cast<double>(i)/1000 + map_point_x[0];
    map_point.y = map_point_y[3];
    map_points.push_back(map_point);
  }
  //右奥正面向きのフェンス
  for(int i=0; i<=int((map_point_y[1] - map_point_y[0])*1000); i++){
    map_point.x = map_point_x[2];
    map_point.y = static_cast<double>(i)/1000 + map_point_y[0];
    map_points.push_back(map_point);
  }
  //右縦向きのフェンス
  for(int i=0; i<=int((map_point_x[2] - map_point_x[1])*1000); i++){
    map_point.x = static_cast<double>(i)/1000 + map_point_x[1];
    map_point.y = map_point_y[1];
    map_points.push_back(map_point);
  }
  //正面のフェンス
  for(int i=0; i<=int((map_point_y[2] - map_point_y[1])*1000); i++){
    map_point.x = map_point_x[1];
    map_point.y = static_cast<double>(i)/1000 + map_point_y[1];
    map_points.push_back(map_point);
  }
  //左縦向きのフェンス
  for(int i=0; i<=int((map_point_x[2] - map_point_x[1])*1000); i++){
    map_point.x = static_cast<double>(i)/1000 + map_point_x[1];
    map_point.y = map_point_y[2];
    map_points.push_back(map_point);
  }
  for(int i=0; i<=int((map_point_y[3] - map_point_y[2])*1000); i++){
    map_point.x = map_point_x[2];
    map_point.y = static_cast<double>(i)/1000 + map_point_y[2];
    map_points.push_back(map_point);
  }
  map_cloud = converter.vector_to_PC2(map_points);
}

// 点群を並進・回転させる
config::LaserPoint IcpBaseSlam::rotate(config::LaserPoint point, double theta){
  config::LaserPoint p;
  p.x = point.x * cos(theta) - point.y * sin(theta);
  p.y = point.x * sin(theta) + point.y * cos(theta);
  return p;
}
vector<config::LaserPoint> IcpBaseSlam::transform(const vector<config::LaserPoint> &points, const Pose &pose) {
  vector<config::LaserPoint> transformed_points;
  for (const auto& point : points) {
    // 並進
    config::LaserPoint p = point;
    p.x += pose.x;
    p.y += pose.y;
    // 回転
    p = rotate(p, pose.yaw);
    transformed_points.push_back(p);
  }
  return transformed_points;
}

}
