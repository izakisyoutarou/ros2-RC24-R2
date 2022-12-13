#include "icp_base_slam/icp_base_slam.hpp"

namespace self_localization{
IcpBaseSlam::IcpBaseSlam(const rclcpp::NodeOptions &options) : IcpBaseSlam("", options) {}

IcpBaseSlam::IcpBaseSlam(const std::string& name_space, const rclcpp::NodeOptions &options)
:  rclcpp::Node("icp_base_slam", name_space, options) {
  RCLCPP_INFO(this->get_logger(), "START");
  plot_mode_ = this->get_parameter("plot_mode").as_bool();
  use_gazebo_simulator_ = this->get_parameter("use_gazebo_simulator").as_bool();
  registration_method_ = this->get_parameter("registration_method").as_string();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  laser_weight_ = this->get_parameter("laser_weight").as_double();
  odom_weight_ = this->get_parameter("odom_weight").as_double();

  scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "scan", rclcpp::SensorDataQoS(),
  bind(&IcpBaseSlam::scan_callback, this, placeholders::_1));

  simulator_odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
    "gazebo_simulator/odom",
    rclcpp::SensorDataQoS(),
    std::bind(&IcpBaseSlam::simulator_odom_callback, this, std::placeholders::_1));

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

  pointcloud2_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
    "self_localization/filtered_cloud",rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  map_pointcloud2_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
    "self_localization/map_point_cloud",rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_publisher = create_publisher<nav_msgs::msg::Path>(
    "self_localization/path",rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  pose_publisher = create_publisher<geometry_msgs::msg::PoseStamped>(
    "self_localization/pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  cloud.points.resize(view_ranges/reso);
  input_elephant_cloud.points.resize(50000);

  if(registration_method_ == "NDT"){
    RCLCPP_INFO(this->get_logger(), "method NDT");
    ndt.setTransformationEpsilon(0.05);
    ndt.setResolution(1.7);
    ndt.setStepSize (0.1);    //ニュートン法のステップサイズ
  }
  else{
    RCLCPP_INFO(this->get_logger(), "method NDT2D");
    ndt2d.setMaximumIterations (1);
    ndt2d.setGridCentre (Eigen::Vector2f (-6.0,-6.0));
    ndt2d.setGridExtent (Eigen::Vector2f (7.0,13.0));
    ndt2d.setGridStep (Eigen::Vector2f (1.7,1.7));
    ndt2d.setOptimizationStepSize (Eigen::Vector3d (0.3,0.3,0.1));
    ndt2d.setTransformationEpsilon (0.1);
  }
  voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

  create_elephant_map();
  init.x = init_pose_x;
  init.y = init_pose_y;
  if(use_gazebo_simulator_){
    last_scan_odom = init;
  }
  else{
    odom = init;
    last_scan_odom = init;
    estimated_odom = init;
  }
  last_estimated = init;
}

void IcpBaseSlam::callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
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
  estimated_odom.x = odom.x + diff_estimated.x;
  estimated_odom.y = odom.y + diff_estimated.y;
  if(plot_mode_) path_view(estimated_odom, msg);
}

void IcpBaseSlam::callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  double yaw = (double)bytes_to_float(_candata);
  diff_odom.yaw = yaw - last_odom.yaw;
  odom.yaw += diff_odom.yaw;
  last_odom.yaw = odom.yaw;
  estimated_odom.yaw = normalize_yaw(odom.yaw + diff_estimated.yaw);
  RCLCPP_INFO(this->get_logger(), "jy yaw->%f°", radToDeg(estimated_odom.yaw));
}

void IcpBaseSlam::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  scan_execution_time_start = chrono::system_clock::now();
  if(scan_execution_time>25){
    RCLCPP_WARN(this->get_logger(), "skip");
    scan_execution_time=0;
    return;
  }
  double current_scan_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_scan = current_scan_received_time - last_scan_received_time;
  last_scan_received_time = current_scan_received_time;
  if (dt_scan > 0.03 /* [sec] */) {
    // RCLCPP_WARN(this->get_logger(), "scan time interval is too large->%f", dt_scan);
  }
  current_scan_odom = estimated_odom;
  Pose scan_odom_motion = current_scan_odom - last_scan_odom; //前回scanからのオドメトリ移動量
  Pose predict = last_estimated + scan_odom_motion;  //前回の推定値にscan間オドメトリ移動量を足し、予測位置を計算

  //予測位置を基準にndtを実行
  double odom_to_lidar_length = 0.4655;
  double odom_to_lidar_x = odom_to_lidar_length * cos(predict.yaw);
  double odom_to_lidar_y = odom_to_lidar_length * sin(predict.yaw);

  for(size_t i=0; i< msg->ranges.size(); ++i) {
    if(msg->ranges[i] > 30 || msg->ranges[i] < 0){msg->ranges[i] = 0;}
    cloud.points[i].x = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i + predict.yaw) + predict.x + odom_to_lidar_x;
    cloud.points[i].y = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i + predict.yaw) + predict.y + odom_to_lidar_y;
  }

  PclCloud::Ptr filtered_cloud_ptr(new PclCloud());
  PclCloud::Ptr cloud_ptr(new PclCloud(cloud));
  voxel_grid_filter.setInputCloud(cloud_ptr);
  voxel_grid_filter.filter(*filtered_cloud_ptr);

  PclCloud result;
  Eigen::Matrix4d transformation_matrix;
  align_time_start = chrono::system_clock::now();
  if(registration_method_ == "NDT"){
    ndt.setInputSource(filtered_cloud_ptr);
    ndt.align(result);
    transformation_matrix = ndt.getFinalTransformation().cast<double>();
    RCLCPP_INFO(this->get_logger(), "TransformationProbability->%f", ndt.getTransformationProbability());
  }
  else{
    ndt2d.setInputSource(filtered_cloud_ptr);
    ndt2d.align(result);
    transformation_matrix = ndt2d.getFinalTransformation().cast<double>();
  }
  align_time_end = chrono::system_clock::now();

  Pose scan_trans;
  scan_trans.x   = transformation_matrix(0,3);
  scan_trans.y   = transformation_matrix(1,3);
  scan_trans.yaw = transformation_matrix.block<3, 3>(0, 0).eulerAngles(0,1,2)(2);
  ndt_estimated = predict + scan_trans;
  ndt_estimated.yaw = normalize_yaw(ndt_estimated.yaw);

  Pose estimated;
  pose_fuser->fuse_pose(ndt_estimated, scan_odom_motion, predict, dt_scan, filtered_cloud_ptr, transformation_matrix, estimated, laser_weight_, odom_weight_);
  if(ndt.getTransformationProbability() < 2.0) estimated.yaw = current_scan_odom.yaw;
  // RCLCPP_INFO(this->get_logger(), "odom      x->%f y->%f yaw->%f°", odom.x, odom.y, radToDeg(odom.yaw));
  // RCLCPP_INFO(this->get_logger(), "odom      x->%f y->%f yaw->%f°", odom.x, odom.y, radToDeg(odom.yaw));
  RCLCPP_INFO(this->get_logger(), "estimated x->%0.3f y->%0.3f yaw->%0.3f°", estimated.x, estimated.y, radToDeg(estimated.yaw));
  diff_estimated = estimated - current_scan_odom;
  last_estimated = estimated;
  last_scan_odom = current_scan_odom;
  if(plot_mode_) pointcloud2_view(filtered_cloud_ptr, input_elephant_cloud, estimated);

  scan_execution_time_end = chrono::system_clock::now();
  scan_execution_time = chrono::duration_cast<chrono::milliseconds>(scan_execution_time_start-scan_execution_time_end).count();
  RCLCPP_INFO(this->get_logger(), "scan execution time->%d", scan_execution_time);
  // RCLCPP_INFO(this->get_logger(), "align time         ->%d", chrono::duration_cast<chrono::milliseconds>(align_time_end-align_time_start).count());
  // RCLCPP_INFO(this->get_logger(), "fuse time          ->%d", chrono::duration_cast<chrono::milliseconds>(time_end-time_start).count());
}

void IcpBaseSlam::simulator_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  double current_odom_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_odom = current_odom_received_time - last_odom_received_time;
  last_odom_received_time = current_odom_received_time;
  if (dt_odom > 0.03 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
  }
  double yaw = quaternionToYaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  diff_odom.x = msg->pose.pose.position.x - last_odom.x;
  diff_odom.y = msg->pose.pose.position.y - last_odom.y;
  diff_odom.yaw = yaw - last_odom.yaw;
  odom += diff_odom;
  last_odom = odom;
  estimated_odom = odom + diff_estimated;
  // RCLCPP_INFO(this->get_logger(), "odom           x->%f y->%f yaw->%f", odom.x, odom.y, odom.yaw);
  // RCLCPP_INFO(this->get_logger(), "estimated_odom x->%f y->%f yaw->%f", estimated_odom.x, estimated_odom.y, estimated_odom.yaw);
  if(plot_mode_) path_view_from_simulator(estimated_odom, msg);
}

double IcpBaseSlam::quaternionToYaw(double x, double y, double z, double w){
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  return atan2(siny_cosp, cosy_cosp);
}

void IcpBaseSlam::print4x4Matrix (const Eigen::Matrix4d & matrix){
  RCLCPP_INFO(this->get_logger(), "    | %3.6f %3.6f %3.6f |",   matrix(0,0), matrix(0,1), matrix(0,2));
  RCLCPP_INFO(this->get_logger(), "R = | %3.6f %3.6f %3.6f |",   matrix(1,0), matrix(1,1), matrix(1,2));
  RCLCPP_INFO(this->get_logger(), "    | %3.6f %3.6f %3.6f |",   matrix(2,0), matrix(2,1), matrix(2,2));
  RCLCPP_INFO(this->get_logger(), "t = < %3.6f, %3.6f, %3.6f >", matrix(0,3), matrix(1,3), matrix(2,3));
}
void IcpBaseSlam::path_view(const Pose &estimate_point, const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(estimate_point.yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
  corrent_pose_stamped.header.stamp = msg->header.stamp;
  corrent_pose_stamped.header.frame_id = "map";
  corrent_pose_stamped.pose.position.x = estimate_point.x;
  corrent_pose_stamped.pose.position.y = estimate_point.y;
  corrent_pose_stamped.pose.position.z = 0.0;
  corrent_pose_stamped.pose.orientation = quat_msg;

  path.header.stamp = msg->header.stamp;
  path.header.frame_id = "map";
  path.poses.push_back(corrent_pose_stamped);
  // RCLCPP_INFO(this->get_logger(), "t->%d x->%3.3f y->%3.3f", msg->header.stamp.sec, corrent_pose_stamped.pose.position.x , corrent_pose_stamped.pose.position.y);
  pose_publisher->publish(corrent_pose_stamped);
  path_publisher->publish(path);
}

void IcpBaseSlam::path_view_from_simulator(const Pose &estimate_point, const nav_msgs::msg::Odometry::SharedPtr msg){
  Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(estimate_point.yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
  corrent_pose_stamped.header.stamp = msg->header.stamp;
  corrent_pose_stamped.header.frame_id = "map";
  corrent_pose_stamped.pose.position.x = estimate_point.x;
  corrent_pose_stamped.pose.position.y = estimate_point.y;
  corrent_pose_stamped.pose.position.z = 0.0;
  corrent_pose_stamped.pose.orientation = quat_msg;

  path.header.stamp = msg->header.stamp;
  path.header.frame_id = "map";
  path.poses.push_back(corrent_pose_stamped);
  // RCLCPP_INFO(this->get_logger(), "t->%d x->%3.3f y->%3.3f", msg->header.stamp.sec, corrent_pose_stamped.pose.position.x , corrent_pose_stamped.pose.position.y);
  pose_publisher->publish(corrent_pose_stamped);
  path_publisher->publish(path);
}

void IcpBaseSlam::pointcloud2_view(PclCloud::Ptr cloud_ptr, PclCloud map_cloud, const Pose estimated){
  sensor_msgs::msg::PointCloud2::SharedPtr msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(*cloud_ptr, *msg_ptr);
  msg_ptr->header.frame_id = "map";
  pointcloud2_publisher->publish(*msg_ptr);
  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(map_cloud, *map_msg_ptr);
  map_msg_ptr->header.frame_id = "map";

  map_pointcloud2_publisher->publish(*map_msg_ptr);
}

void IcpBaseSlam::create_elephant_map(){
  double rafter_length = 12.;
  double fence_width = 0.025;
  double rafter_x_max = (rafter_length - fence_width)/2;
  double rafter_y_max = rafter_length - rafter_width;
  double front_fence_x = 2.0;
  double fence_right_y = 2.0;
  double fence_left_y = 10.0;

  int count=0;
  int count_sum=0;
  //右の垂木
  for(int i=0; i<=int((rafter_x_max - rafter_width)*1000); i++){
    input_elephant_cloud.points[i].x = double(i)/1000 + rafter_width;
    input_elephant_cloud.points[i].y = rafter_width;
    count++;
  }
  count_sum=count;
  //後ろの垂木
  for(int i=0; i<=int((rafter_y_max - rafter_width)*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = rafter_width;
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + rafter_width;
    count++;
  }
  count_sum=count;
  //左の垂木
  for(int i=0; i<=int((rafter_x_max - rafter_width)*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = double(i)/1000 + rafter_width;
    input_elephant_cloud.points[i+count_sum].y = rafter_y_max;
    count++;
  }
  count_sum=count;
  //右奥正面向きのフェンス
  for(int i=0; i<=int((fence_right_y - rafter_width)*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = rafter_x_max;
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + rafter_width;
    count++;
  }
  count_sum=count;
  //右縦向きのフェンス
  for(int i=0; i<=int((rafter_x_max - front_fence_x)*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = double(i)/1000 + front_fence_x;
    input_elephant_cloud.points[i+count_sum].y = fence_right_y;
    count++;
  }
  count_sum=count;
  //正面のフェンス
  for(int i=0; i<=int((fence_left_y - fence_right_y)*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = front_fence_x;
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + fence_right_y;
    count++;
  }
  count_sum=count;
  //左縦向きのフェンス
  for(int i=0; i<=int((rafter_x_max - front_fence_x)*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = double(i)/1000 + front_fence_x;
    input_elephant_cloud.points[i+count_sum].y = fence_left_y;
    count++;
  }
  count_sum=count;
  for(int i=0; i<=int((rafter_y_max - fence_left_y)*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = rafter_x_max;
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + fence_left_y;
    count++;
  }
  //原点を合わせる
  for(size_t i=0; i<input_elephant_cloud.size(); i++){
    input_elephant_cloud.points[i].x -= 6.0;
    input_elephant_cloud.points[i].y -= 6.0;
  }

  if(registration_method_ == "NDT"){
    ndt.setInputTarget(input_elephant_cloud.makeShared());
  }
  else{
    pcl::VoxelGrid<PointType> voxel_grid_filter_map;
    pcl::PointCloud<PointType>::Ptr filtered_elephant_cloud_ptr(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr input_elephant_cloud_ptr(new pcl::PointCloud<PointType>(input_elephant_cloud));
    voxel_grid_filter_map.setLeafSize(0.3, 0.3, 0.3);
    voxel_grid_filter_map.setInputCloud(input_elephant_cloud_ptr);
    voxel_grid_filter_map.filter(*filtered_elephant_cloud_ptr);
    ndt2d.setInputTarget(filtered_elephant_cloud_ptr);
  }
}


void IcpBaseSlam::make_input_circles(){
  input_circle_cloud.points.resize(view_ranges/reso);
  for(int i=0; i<600; i++){
    if(i>=0 && i<100){
      input_circle_cloud.points[i].x = circle_model_x(i, model_count);
      input_circle_cloud.points[i].y = circle_model_y_dec(input_circle_cloud.points[i].x, circle_right_y);
    }
    else if(i>=100 && i<200){
      model_count=1;
      input_circle_cloud.points[i].x = circle_model_x(i, model_count);
      input_circle_cloud.points[i].y = circle_model_y_inc(input_circle_cloud.points[i].x, circle_right_y);
    }
    else if(i>=200 && i<300){
      model_count=2;
      input_circle_cloud.points[i].x = circle_model_x(i, model_count);
      input_circle_cloud.points[i].y = circle_model_y_dec(input_circle_cloud.points[i].x, circle_center_y);
    }
    else if(i>=300 && i<400){
      model_count=3;
      input_circle_cloud.points[i].x = circle_model_x(i, model_count);
      input_circle_cloud.points[i].y = circle_model_y_inc(input_circle_cloud.points[i].x, circle_center_y);
    }
    else if(i>=400 && i<500){
      model_count=4;
      input_circle_cloud.points[i].x = circle_model_x(i, model_count);
      input_circle_cloud.points[i].y = circle_model_y_dec(input_circle_cloud.points[i].x, circle_left_y);
    }
    else if(i>=500 && i<600){
      model_count=5;
      input_circle_cloud.points[i].x = circle_model_x(i, model_count);
      input_circle_cloud.points[i].y = circle_model_y_inc(input_circle_cloud.points[i].x, circle_left_y);
    }
  }
}

double IcpBaseSlam::circle_model_x(int i, int model_count){
  return circle_x - R + double(i-model_count*100)/1000;
}

double IcpBaseSlam::circle_model_y_inc(double point_x, double circle_y){
  return circle_y + sqrt(pow(R,2) - pow(point_x - circle_x, 2));
}

double IcpBaseSlam::circle_model_y_dec(double point_x, double circle_y){
  return circle_y - sqrt(pow(R,2) - pow(point_x - circle_x, 2));
}

int IcpBaseSlam::max_time(int num){
  if(num > last_num_time){
    last_num_time = num;
  }
  return last_num_time;
}

int IcpBaseSlam::max_iteration(int num){
  if(num > last_num_iteration){
    last_num_iteration = num;
  }
  return last_num_iteration;
}
}
