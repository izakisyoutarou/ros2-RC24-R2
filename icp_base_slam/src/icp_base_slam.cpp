#include "icp_base_slam/icp_base_slam.hpp"

namespace self_localization{
IcpBaseSlam::IcpBaseSlam(const rclcpp::NodeOptions &options) : IcpBaseSlam("", options) {}

IcpBaseSlam::IcpBaseSlam(const std::string& name_space, const rclcpp::NodeOptions &options)
:  rclcpp::Node("icp_base_slam", name_space, options) {
  RCLCPP_INFO(this->get_logger(), "START");
  plot_mode_ = this->get_parameter("plot_mode").as_bool();
  use_gazebo_simulator_ = this->get_parameter("use_gazebo_simulator").as_bool();
  registration_method_ = this->get_parameter("registration_method").as_string();
  filtering_method_ = this->get_parameter("filtering_method").as_string();
  voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
  laser_weight_ = this->get_parameter("laser_weight").as_double();
  odom_weight_ = this->get_parameter("odom_weight").as_double();
  trial_num_ = this->get_parameter("trial_num").as_int();
  inlier_dist_threshold_ = this->get_parameter("inlier_dist_threshold").as_double();
  auto transformation_epsilon_ = this->get_parameter("transformation_epsilon").as_double();
  auto resolution_ = this->get_parameter("resolution").as_double();
  auto step_size_ = this->get_parameter("step_size").as_double();

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

  pointcloud2_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "self_localization/filtered_cloud",rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  map_pointcloud2_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "self_localization/map_point_cloud",rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  ransac_pointcloud2_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "self_localization/ransac_point_cloud",rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  path_publisher = this->create_publisher<nav_msgs::msg::Path>(
    "self_localization/path",rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "self_localization/pose", rclcpp::QoS(rclcpp::KeepLast(0)).transient_local().reliable());

  cloud.points.resize(view_ranges/reso);
  input_elephant_cloud.points.resize(51655);
  inlier_cloud.points.resize(2161);

  // RCLCPP_INFO(this->get_logger(), "method NDT");
  ndt.setTransformationEpsilon(transformation_epsilon_);
  ndt.setResolution(resolution_);
  ndt.setStepSize (step_size_);    //ニュートン法のステップサイズ

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
  // RCLCPP_INFO(this->get_logger(), "odom x->%f y->%f", odom.x, odom.y);
  // RCLCPP_INFO(this->get_logger(), "diff_estimated x->%f y->%f", diff_estimated.x, diff_estimated.y);
  // RCLCPP_INFO(this->get_logger(), "estimated_odom x->%f y->%f", estimated_odom.x, estimated_odom.y);
}

void IcpBaseSlam::callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  double yaw = (double)bytes_to_float(_candata);
  diff_odom.yaw = yaw - last_odom.yaw;
  odom.yaw += diff_odom.yaw;
  last_odom.yaw = odom.yaw;
  estimated_odom.yaw = normalize_yaw(odom.yaw + diff_estimated.yaw);
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
  Pose estimated;
  current_scan_odom = estimated_odom;
  Pose scan_odom_motion = current_scan_odom - last_scan_odom; //前回scanからのオドメトリ移動量
  Pose predict = last_estimated + scan_odom_motion;  //前回の推定値にscan間オドメトリ移動量を足し、予測位置を計算

  //予測位置を基準にndtを実行
  double odom_to_lidar_length = 0.4655;
  double odom_to_lidar_x = odom_to_lidar_length * cos(current_scan_odom.yaw);
  double odom_to_lidar_y = odom_to_lidar_length * sin(current_scan_odom.yaw);
  vector<config::LaserPoint> src_points;
  for(size_t i=0; i< msg->ranges.size(); ++i) {
    config::LaserPoint src_point;
    if(msg->ranges[i] > 14 || msg->ranges[i] < 0.5){msg->ranges[i] = 0;}
    src_point.x = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i - current_scan_odom.yaw) + current_scan_odom.x + odom_to_lidar_x;
    src_point.y = -msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i - current_scan_odom.yaw) + current_scan_odom.y + odom_to_lidar_y;
    src_point.id = i;
    src_points.push_back(src_point);
  }


  if(registration_method_ == "ransac"){
    ransac_lines->fuse_inliers(src_points, trial_num_, inlier_dist_threshold_, estimated_odom);
    vector<config::LaserPoint> line_points = ransac_lines->get_sum();
    diff_estimated=ransac_lines->get_estimated_diff();
    RCLCPP_INFO(this->get_logger(), "diff_estimated yaw->%f", radToDeg(ransac_lines->get_estimated_diff().yaw));
    inlier_cloud.points.resize(line_points.size());
    for(size_t i=0; i<line_points.size(); i++){
      inlier_cloud.points[i].x = line_points[i].x;
      inlier_cloud.points[i].y = line_points[i].y;
    }
  }

  else if(registration_method_ == "NDT"){
    // ransac_lines->fuse_inliers(cloud, trial_num_, inlier_dist_threshold_);
    vector<config::LaserPoint> line_points = ransac_lines->get_sum();
    PclCloud filtered_cloud;

    if(filtering_method_=="dynamic_voxel_grid_filter"){
      vector<config::LaserPoint> filtered = dynamic_voxel_grid_filter.variable_voxel(line_points);
      filtered_cloud.points.resize(filtered.size());
      for(size_t i=0; i<filtered_cloud.points.size(); i++){
        filtered_cloud.points[i].x = filtered[i].x;
        filtered_cloud.points[i].y = filtered[i].y;
        // RCLCPP_INFO(this->get_logger(), "filtered x->%f y->%f", filtered_cloud.points[i].x, filtered_cloud.points[i].y);
      }
    }
    else if(filtering_method_=="voxel_grid_filter"){
      inlier_cloud.points.resize(line_points.size());
      for(size_t i=0; i<line_points.size(); i++){
        inlier_cloud.points[i].x = line_points[i].x;
        inlier_cloud.points[i].y = line_points[i].y;
      }
      voxel_grid_filter.setInputCloud(inlier_cloud.makeShared());
      voxel_grid_filter.filter(filtered_cloud);
    }

    PclCloud result;
    Eigen::Matrix4d transformation_matrix;
    align_time_start = chrono::system_clock::now();
    ndt.setInputSource(filtered_cloud.makeShared());
    ndt.align(result);
    transformation_matrix = ndt.getFinalTransformation().cast<double>();
    RCLCPP_INFO(this->get_logger(), "TransformationProbability->%f", ndt.getTransformationProbability());

    align_time_end = chrono::system_clock::now();

    Pose scan_trans;
    scan_trans.x   = transformation_matrix(0,3);
    scan_trans.y   = transformation_matrix(1,3);
    scan_trans.yaw = transformation_matrix.block<3, 3>(0, 0).eulerAngles(0,1,2)(2);
    ndt_estimated = predict + scan_trans;
    ndt_estimated.yaw = normalize_yaw(ndt_estimated.yaw);
    PclCloud::Ptr filtered_cloud_ptr(new PclCloud(filtered_cloud));
    pose_fuser->fuse_pose(ndt_estimated, scan_odom_motion, predict, dt_scan, filtered_cloud_ptr, transformation_matrix, estimated, laser_weight_, odom_weight_);
    if(ndt.getTransformationProbability() < 3.0) estimated.yaw = current_scan_odom.yaw;

    // RCLCPP_INFO(this->get_logger(), "odom      x->%f y->%f yaw->%f°", odom.x, odom.y, radToDeg(odom.yaw));
    // RCLCPP_INFO(this->get_logger(), "odom      x->%f y->%f yaw->%f°", odom.x, odom.y, radToDeg(odom.yaw));
    RCLCPP_INFO(this->get_logger(), "estimated x->%0.3f y->%0.3f yaw->%0.3f°", estimated.x, estimated.y, radToDeg(estimated.yaw));
    RCLCPP_INFO(this->get_logger(), "align time         ->%d", chrono::duration_cast<chrono::milliseconds>(align_time_end-align_time_start).count());
    RCLCPP_INFO(this->get_logger(), "time          ->%d", chrono::duration_cast<chrono::milliseconds>(time_end-time_start).count());
  }
  // diff_estimated = estimated - current_scan_odom;
  // last_estimated = estimated;
  // last_scan_odom = current_scan_odom;
  scan_execution_time_end = chrono::system_clock::now();
  scan_execution_time = chrono::duration_cast<chrono::milliseconds>(scan_execution_time_end-scan_execution_time_start).count();
  // RCLCPP_INFO(this->get_logger(), "scan execution time->%d", scan_execution_time);
  if(plot_mode_) pointcloud2_view(input_elephant_cloud, inlier_cloud);
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
  pose_publisher->publish(corrent_pose_stamped);
  path_publisher->publish(path);
}

void IcpBaseSlam::pointcloud2_view(PclCloud &map_cloud, PclCloud &ransac_cloud){
  sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
  sensor_msgs::msg::PointCloud2::SharedPtr ransac_msg_ptr(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(map_cloud, *map_msg_ptr);
  pcl::toROSMsg(cloud_add_collor(ransac_cloud, "g"), *ransac_msg_ptr);
  map_msg_ptr->header.frame_id = "map";
  ransac_msg_ptr->header.frame_id = "map";
  map_pointcloud2_publisher->publish(*map_msg_ptr);
  ransac_pointcloud2_publisher->publish(*ransac_msg_ptr);

  Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(estimated_odom.yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
  corrent_pose_stamped.header.stamp = this->now();
  corrent_pose_stamped.header.frame_id = "map";
  corrent_pose_stamped.pose.position.x = estimated_odom.x;
  corrent_pose_stamped.pose.position.y = estimated_odom.y;
  corrent_pose_stamped.pose.position.z = 0.0;
  corrent_pose_stamped.pose.orientation = quat_msg;

  path.header.stamp = this->now();
  path.header.frame_id = "map";
  path.poses.push_back(corrent_pose_stamped);
  pose_publisher->publish(corrent_pose_stamped);
  path_publisher->publish(path);
}

pcl::PointCloud<pcl::PointXYZRGB> IcpBaseSlam::cloud_add_collor(PclCloud &cloud, char *rgb){
  pcl::PointCloud<pcl::PointXYZRGB> cloud_collor;
  char collor = *rgb;
  for(size_t i=0; i<cloud.points.size(); i++){
    pcl::PointXYZRGB new_point;
    new_point.x = cloud.points[i].x;
    new_point.y = cloud.points[i].y;
    switch(collor){
      case 'r':
        new_point.r = 255;
        break;
      case 'g':
        new_point.g = 255;
        break;
      case 'b':
        new_point.b = 255;
        break;
      }
    cloud_collor.points.push_back(new_point);
  }
  return cloud_collor;
}

void IcpBaseSlam::create_elephant_map(){
  int count=0;
  int count_sum=0;
  //右の垂木
  for(int i=0; i<=int((map_point_x[2] - map_point_x[0])*1000); i++){
    input_elephant_cloud.points[i].x = double(i)/1000 + map_point_x[0];
    input_elephant_cloud.points[i].y = map_point_y[0];
    count++;
  }
  count_sum=count;
  //後ろの垂木
  for(int i=0; i<=int((map_point_y[3] - map_point_y[0])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = map_point_x[0];
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + map_point_y[0];
    count++;
  }
  count_sum=count;
  //左の垂木
  for(int i=0; i<=int((map_point_x[2] - map_point_x[0])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = double(i)/1000 + map_point_x[0];
    input_elephant_cloud.points[i+count_sum].y = map_point_y[3];
    count++;
  }
  count_sum=count;
  //右奥正面向きのフェンス
  for(int i=0; i<=int((map_point_y[1] - map_point_y[0])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = map_point_x[2];
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + map_point_y[0];
    count++;
  }
  count_sum=count;
  //右縦向きのフェンス
  for(int i=0; i<=int((map_point_x[2] - map_point_x[1])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = double(i)/1000 + map_point_x[1];
    input_elephant_cloud.points[i+count_sum].y = map_point_y[1];
    count++;
  }
  count_sum=count;
  //正面のフェンス
  for(int i=0; i<=int((map_point_y[2] - map_point_y[1])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = map_point_x[1];
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + map_point_y[1];
    count++;
  }
  count_sum=count;
  //左縦向きのフェンス
  for(int i=0; i<=int((map_point_x[2] - map_point_x[1])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = double(i)/1000 + map_point_x[1];
    input_elephant_cloud.points[i+count_sum].y = map_point_y[2];
    count++;
  }
  count_sum=count;
  for(int i=0; i<=int((map_point_y[3] - map_point_y[2])*1000); i++){
    input_elephant_cloud.points[i+count_sum].x = map_point_x[2];
    input_elephant_cloud.points[i+count_sum].y = double(i)/1000 + map_point_y[2];
    count++;
  }
  ndt.setInputTarget(input_elephant_cloud.makeShared());
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
