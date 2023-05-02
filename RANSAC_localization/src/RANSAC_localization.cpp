#include "RANSAC_localization/RANSAC_localization.hpp"

namespace self_localization{
RANSACLocalization::RANSACLocalization(const rclcpp::NodeOptions &options) : RANSACLocalization("", options) {}

RANSACLocalization::RANSACLocalization(const string& name_space, const rclcpp::NodeOptions &options)
:  rclcpp::Node("RANSAC_localization", name_space, options){
  RCLCPP_INFO(this->get_logger(), "START");
  robot_type_ = this->get_parameter("robot_type").as_string();
  plot_mode_ = this->get_parameter("plot_mode").as_bool();
  const auto pose_array = this->get_parameter("initial_pose").as_double_array();
  const auto tf_array = this->get_parameter("tf_laser2robot").as_double_array();
  const auto laser_weight_ = this->get_parameter("laser_weight").as_double();
  const auto odom_weight_liner_ = this->get_parameter("odom_weight_liner").as_double();
  const auto odom_weight_angler_ = this->get_parameter("odom_weight_angler").as_double();
  const auto voxel_size_ = this->get_parameter("voxel_size").as_double();
  const auto trial_num_ = this->get_parameter("trial_num").as_int();
  const auto inlier_dist_threshold_ = this->get_parameter("inlier_dist_threshold").as_double();
  const auto inlier_length_threshold_ = this->get_parameter("inlier_length_threshold").as_double();

  restart_subscriber = this->create_subscription<controller_interface_msg::msg::BaseControl>(
    "pub_base_control",_qos,
    bind(&RANSACLocalization::callback_restart, this, placeholders::_1));

  scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
  bind(&RANSACLocalization::callback_scan, this, placeholders::_1));

  odom_linear_subscriber = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
    "can_rx_110",_qos,
    bind(&RANSACLocalization::callback_odom_linear, this, placeholders::_1));

  odom_angular_subscriber = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
    "can_rx_111",_qos,
    bind(&RANSACLocalization::callback_odom_angular, this, placeholders::_1));

  init_angle_publisher = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>(
    "can_tx_120",_qos);

  self_pose_publisher = this->create_publisher<geometry_msgs::msg::Vector3>(
    "self_pose", _qos);

  detect_lines.setup(robot_type_, voxel_size_, trial_num_, inlier_dist_threshold_, inlier_length_threshold_);
  pose_fuser.setup(robot_type_, laser_weight_, odom_weight_liner_, odom_weight_angler_);
  voxel_grid_filter.setup(voxel_size_);

  tf_laser2robot << tf_array[0], tf_array[1], tf_array[2], tf_array[3], tf_array[4], tf_array[5];
  init_pose << pose_array[0], pose_array[1], pose_array[2];
  init();

  if(plot_mode_){
    ransaced_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "self_localization/ransac",fast_qos);

    path_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "self_localization/path",fast_qos);

    pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "self_localization/pose", fast_qos);

    map_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "self_localization/map", fast_qos);

    if(robot_type_ == "ER") create_ER_map();
    else if(robot_type_ == "RR") create_RR_map();
  }
}

void RANSACLocalization::init(){
  odom = Vector3d::Zero();
  last_odom = Vector3d::Zero();
  est_diff_sum = init_pose;
  last_estimated = init_pose;
  pose_fuser.init();
  detect_lines.init();
  detect_circles.init();
}

void RANSACLocalization::callback_restart(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "RESTART");
  if(msg->is_restart){
    init();
    // 初期角度をpublish
    uint8_t _candata[8];
    auto msg_angle = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_angle->canid = 0x120;
    msg_angle->candlc = 4;
    float_to_bytes(_candata, static_cast<float>(init_pose[2]));
    for(int i=0; i<msg_angle->candlc; i++) msg_angle->candata[i] = _candata[i];
    init_angle_publisher->publish(*msg_angle);
  }
}

void RANSACLocalization::callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  double odom_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_odom = odom_received_time - last_odom_received_time;
  last_odom_received_time = odom_received_time;
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  const double x = (double)bytes_to_float(_candata);
  const double y = (double)bytes_to_float(_candata+4);
  Vector3d diff_odom = Vector3d::Zero();
  diff_odom[0] = x - last_odom[0];
  diff_odom[1] = y - last_odom[1];

  if(abs(diff_odom[0]) / dt_odom > 4) diff_odom[0] = 0.0;
  if(abs(diff_odom[1]) / dt_odom > 4) diff_odom[1] = 0.0;

  odom[0] += diff_odom[0];
  odom[1] += diff_odom[1];

  last_odom[0] = x;
  last_odom[1] = y;

  vector_msg.x = odom[0] + est_diff_sum[0];
  vector_msg.y = odom[1] + est_diff_sum[1];
}

void RANSACLocalization::callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  double jy_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_jy = jy_received_time - last_jy_received_time;
  last_jy_received_time = jy_received_time;
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  const double yaw = (double)bytes_to_float(_candata);
  odom[2] = yaw;
  if(abs(odom[2] - last_odom[2]) / dt_jy > 6*M_PI) odom[2] = last_odom[2];
  last_odom[2] = odom[2];
  vector_msg.z = normalize_yaw(odom[2] + est_diff_sum[2]);
  self_pose_publisher->publish(vector_msg);
}

void RANSACLocalization::callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  time_start = chrono::system_clock::now();
  double current_scan_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_scan = current_scan_received_time - last_scan_received_time;
  last_scan_received_time = current_scan_received_time;
  // if (dt_scan > 0.03 /* [sec] */) RCLCPP_WARN(this->get_logger(), "scan time interval is too large->%f", dt_scan);

  Vector3d current_scan_odom = odom + est_diff_sum;
  Vector3d scan_odom_motion = current_scan_odom - last_estimated; //前回scanからのオドメトリ移動量
  if (scan_odom_motion[2] > M_PI) last_estimated[2] += 2*M_PI;
  else if (scan_odom_motion[2] < -M_PI) last_estimated[2] -= 2*M_PI;
  scan_odom_motion[2] = current_scan_odom[2] - last_estimated[2];
  tf_laser2robot[5] = current_scan_odom[2];

  Vector3d laser = current_scan_odom + calc_body_to_sensor(tf_laser2robot);

  vector<LaserPoint> src_points = converter.scan_to_vector(msg, laser);
  vector<LaserPoint> filtered_points = voxel_grid_filter.apply_voxel_grid_filter(src_points);

  detect_lines.fuse_inliers(filtered_points);
  vector<LaserPoint> line_points = detect_lines.get_sum();
  Vector3d trans = detect_lines.get_estimated_diff();
  Vector3d ransac_estimated = current_scan_odom + trans;
  vector<LaserPoint> global_points = transform(line_points, trans);
  Vector3d estimated = pose_fuser.fuse_pose(ransac_estimated, scan_odom_motion, current_scan_odom, dt_scan, line_points, global_points);

  update(estimated, ransac_estimated, current_scan_odom, scan_odom_motion, src_points);

  last_estimated = estimated;

  if(plot_mode_) publishers(src_points);
  time_end = chrono::system_clock::now();
  // RCLCPP_INFO(this->get_logger(), "estimated x>%f y>%f a>%f°", estimated[0], estimated[1], radToDeg(estimated[2]));
  // RCLCPP_INFO(this->get_logger(), "trans x>%f y>%f a>%f°", trans[0], trans[1], radToDeg(trans[2]));
  // RCLCPP_INFO(this->get_logger(), "scan time->%d", chrono::duration_cast<chrono::milliseconds>(time_end-time_start).count());
}

void RANSACLocalization::update(const Vector3d &estimated, const Vector3d &laser_estimated, const Vector3d &current_scan_odom, const Vector3d &scan_odom_motion, vector<LaserPoint> &points){
  Vector3d diff_circle = Vector3d::Zero();
  if(robot_type_ == "RR"){
    get_correction_rate_average(estimated, laser_estimated, current_scan_odom);
    diff_circle = correction_rate_ave.cwiseProduct(detect_circles.calc_diff_pose(points));
    diff_circle[2] = 0.0;  //円の半径と角度の掛け算をしたため
  }
  correction(scan_odom_motion, estimated, current_scan_odom, diff_circle);
}

void RANSACLocalization::correction(const Vector3d &scan_odom_motion, const Vector3d &estimated, const Vector3d &current_scan_odom, const Vector3d &diff_circle){
  const double motion_dist = distance(0.0,scan_odom_motion[0],0.0,scan_odom_motion[1]);
  Vector3d est_diff = estimated - current_scan_odom;  //直線からの推定値がデフォルト
  if(motion_dist > 0.015){
    for(size_t i=0; i<2; i++){
      if(est_diff[i] == 0.0 && isfinite(diff_circle[i]) && robot_type_ == "RR") est_diff[i] = diff_circle[i];  //直線からの推定値が0の場合、円から推定
      est_diff_sum[i] += est_diff[i];
    }
  }
  if(abs(scan_odom_motion[2]) > 0.001) est_diff_sum[2] += est_diff[2];
}

void RANSACLocalization::get_correction_rate_average(const Vector3d &estimated, const Vector3d &laser_estimated, const Vector3d &current_scan_odom){
  for(size_t i=0; i<correction_rate_ave.size(); i++){
    double calc_correction_rate_average_ = calc_correction_rate_average(estimated[i], laser_estimated[i], current_scan_odom[i], correction_rate_sum[i], correction_count[i]);
    if(isfinite(calc_correction_rate_average_)) correction_rate_ave[i] = calc_correction_rate_average_;
  }
}

double RANSACLocalization::calc_correction_rate_average(const double &estimated_, const double &laser_estimated_, const double &current_scan_odom_, double &correction_rate_sum_, int &correction_count_){
  double correction_rate_ = (1 - abs((laser_estimated_-estimated_) / (laser_estimated_-current_scan_odom_)));
  if(!(correction_rate_==0.0) && isfinite(correction_rate_)){
    correction_count_++;
    correction_rate_sum_ += correction_rate_;
  }
  return correction_rate_sum_/correction_count_;
}

void RANSACLocalization::publishers(vector<LaserPoint> &points){
  sensor_msgs::msg::PointCloud2 cloud = converter.vector_to_PC2(points);

  corrent_pose_stamped.header.stamp = this->now();
  corrent_pose_stamped.header.frame_id = "map";
  corrent_pose_stamped.pose.position.x = odom[0] + est_diff_sum[0];
  corrent_pose_stamped.pose.position.y = odom[1] + est_diff_sum[1];
  corrent_pose_stamped.pose.orientation.z = sin((odom[2]+est_diff_sum[2]) / 2.0);
  corrent_pose_stamped.pose.orientation.w = cos((odom[2]+est_diff_sum[2]) / 2.0);

  // path.header.stamp = this->now();
  // path.header.frame_id = "map";
  // path.poses.push_back(corrent_pose_stamped);

  if(robot_type_ == "ER") map_publisher->publish(ER_map_cloud);
  else if(robot_type_ == "RR") map_publisher->publish(RR_map_cloud);
  ransaced_publisher->publish(cloud);
  pose_publisher->publish(corrent_pose_stamped);
  // path_publisher->publish(path);
}


void RANSACLocalization::create_ER_map(){
  //右の垂木
  create_map_line(ER_map_points, ER_map_point_x[0], ER_map_point_x[3], ER_map_point_y[0], 'x');
  //後ろの垂木
  create_map_line(ER_map_points, ER_map_point_y[0], ER_map_point_y[3], ER_map_point_x[0], 'y');
  //左の垂木
  create_map_line(ER_map_points, ER_map_point_x[0], ER_map_point_x[2], ER_map_point_y[3], 'x');
  //右奥正面向きのフェンス
  create_map_line(ER_map_points, ER_map_point_y[0], ER_map_point_y[1], ER_map_point_x[3], 'y');
  //右縦向きのフェンス
  create_map_line(ER_map_points, ER_map_point_x[1], ER_map_point_x[3]-bridge_width, ER_map_point_y[1], 'x');
  //正面のフェンス
  create_map_line(ER_map_points, ER_map_point_y[1], ER_map_point_y[2], ER_map_point_x[1], 'y');
  //左縦向きのフェンス
  create_map_line(ER_map_points, ER_map_point_x[1], ER_map_point_x[2], ER_map_point_y[2], 'x');
  //左奥正面向きのフェンス
  create_map_line(ER_map_points, ER_map_point_y[2], ER_map_point_y[3], ER_map_point_x[2], 'y');
  ER_map_cloud = converter.vector_to_PC2(ER_map_points);
}

void RANSACLocalization::create_RR_map(){
  //2段目右
  create_map_line(RR_map_points, RR_map_point[0], RR_map_point[3], RR_map_point[0], 'x');
  //2段目正面
  create_map_line(RR_map_points, RR_map_point[0], RR_map_point[3], RR_map_point[0], 'y');
  //2段目左
  create_map_line(RR_map_points, RR_map_point[0], RR_map_point[3], RR_map_point[3], 'x');
  //3段目右
  create_map_line(RR_map_points, RR_map_point[1], RR_map_point[2], RR_map_point[1], 'x');
  //3段目正面
  create_map_line(RR_map_points, RR_map_point[1], RR_map_point[2], RR_map_point[1], 'y');
  //3段目左
  create_map_line(RR_map_points, RR_map_point[1], RR_map_point[2], RR_map_point[2], 'x');
  generate_circle(RR_map_points, circle_self_right, 2000);
  generate_circle(RR_map_points, circle_self_center, 2000);
  generate_circle(RR_map_points, circle_self_left, 2000);
  generate_circle(RR_map_points, circle_opponent_right, 2000);
  generate_circle(RR_map_points, circle_opponent_left, 2000);
  RR_map_cloud = converter.vector_to_PC2(RR_map_points);
}

void RANSACLocalization::generate_circle(vector<LaserPoint> &points, const Vector3d &circle, int num_points){
  // 半円上の点を生成
  for (int i = 0; i < num_points / 2; ++i){
    LaserPoint point;
    double theta = static_cast<double>(i) / static_cast<double>(num_points / 2 - 1) * M_PI;
    point.x = circle[0] + circle[2] * cos(theta);
    point.y = circle[1] + semi_circle(point.x - circle[0], circle[2]);
    if(i%2==0) point.y = -point.y;
    points.push_back(point);
  }
}


void RANSACLocalization::create_map_line(vector<LaserPoint> &points, const double &start_map_point, const double &end_map_point, const double &static_map_point, const char coordinate){
  LaserPoint map_point;
  for(int i=0; i<=int((end_map_point - start_map_point)*1000); i++){
    if(coordinate == 'x'){
      map_point.x = static_cast<double>(i)/1000 + start_map_point;
      map_point.y = static_map_point;
    }
    else if(coordinate == 'y'){
      map_point.x = static_map_point;
      map_point.y = static_cast<double>(i)/1000 + start_map_point;
    }
    points.push_back(map_point);
  }
}

Vector3d RANSACLocalization::calc_body_to_sensor(const Vector6d& sensor_pos){
  // yaw, pitch, rollから回転行列を計算
  Vector3d sensor_pos_;
  sensor_pos_ << sensor_pos[0], -sensor_pos[1], sensor_pos[2];
  const double s_r = sin(sensor_pos[3]);
  const double s_p = sin(sensor_pos[4]);
  const double s_y = sin(sensor_pos[5]);
  const double c_r = cos(sensor_pos[3]);
  const double c_p = cos(sensor_pos[4]);
  const double c_y = cos(sensor_pos[5]);
  Matrix3d R;
  R << c_y * c_p,  c_y * s_p * s_r - s_y * c_r,  c_y * s_p * c_r + s_y * s_r,
       s_y * c_p,  s_y * s_p * s_r + c_y * c_r,  s_y * s_p * c_r - c_y * s_r,
      -s_p,        c_p * s_r,                    c_p * c_r;
  // センサの座標を回転行列で機体座標系に変換する
  return R * sensor_pos_;
}
}
