#include <pcl_localization/pcl_localization_component.hpp>
PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pcl_localization", options),
  broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);
  declare_parameter("use_odom", false);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializePubSub();
  initializeRegistration();

  path_.header.frame_id = global_frame_id_;

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  pose_pub_->on_activate();
  path_pub_->on_activate();

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.position.x = initial_pose_x_;
    msg->pose.position.y = initial_pose_y_;
    msg->pose.position.z = initial_pose_z_;
    msg->pose.orientation.x = initial_pose_qx_;
    msg->pose.orientation.y = initial_pose_qy_;
    msg->pose.orientation.z = initial_pose_qz_;
    msg->pose.orientation.w = initial_pose_qw_;

    path_.poses.push_back(*msg);

    initialPoseReceived(msg);
  }
  cloud.points.resize(2161);
  input_elephant_cloud.points.resize(50000);
  create_elephant_map();
  RCLCPP_INFO(get_logger(), "Activating end");

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  odom_sub_.reset();
  scan_subscriber.reset();
  imu_sub_.reset();
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);

  RCLCPP_INFO(get_logger(),"global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(),"ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(),"transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(),"voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(),"use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(),"use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(),"use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(),"enable_debug: %d", enable_debug_);
}

void PCLLocalization::initializePubSub(){
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    "pcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  odom_delay_subscriber = this->create_subscription<my_messages::msg::OdomDelay>(
    "odom_delay", 10, bind(&PCLLocalization::odom_delay_callback, this, placeholders::_1));

  scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "scan", rclcpp::SensorDataQoS(),
  std::bind(&PCLLocalization::scanReceived, this, placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));
}

void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");
  ndt.setMaximumIterations(30);
  ndt.setStepSize(ndt_step_size_);
  ndt.setResolution(ndt_resolution_);
  ndt.setTransformationEpsilon(transform_epsilon_);

  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
}

void PCLLocalization::initialPoseReceived(geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  initialpose_recieved_ = true;
  corrent_pose_stamped_ = *msg;
  pose_pub_->publish(corrent_pose_stamped_);
}



void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!use_odom_) {return;}

  double current_odom_received_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;
  double dt_odom = current_odom_received_time - last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;
  if (dt_odom > 1.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
    return;
  }
  if (dt_odom < 0.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
    return;
  }

  tf2::Quaternion previous_quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(corrent_pose_stamped_.pose.orientation, previous_quat_tf);
  tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

  roll += msg->twist.twist.angular.x * dt_odom;
  pitch += msg->twist.twist.angular.y * dt_odom;
  yaw += msg->twist.twist.angular.z * dt_odom;
  // RCLCPP_INFO(this->get_logger(), "yaw->%f", radToDeg(yaw));
  Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  Eigen::Vector3d odom{
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z};
  Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;

  corrent_pose_stamped_.pose.position.x += delta_position.x();
  corrent_pose_stamped_.pose.position.y += delta_position.y();
  corrent_pose_stamped_.pose.position.z += delta_position.z();
  corrent_pose_stamped_.pose.orientation = quat_msg;
}

void PCLLocalization::odom_delay_callback(const my_messages::msg::OdomDelay::SharedPtr msg){
  int receive_time = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
  int dt_odom = receive_time - last_receive_time;
  last_receive_time = receive_time;
  double vel_x = (msg->x - last_pose_x)/dt_odom;
  double vel_y = (msg->y - last_pose_y)/dt_odom;
  double vel_yaw = (msg->yaw - last_pose_yaw)/dt_odom;
  last_pose_x = msg->x;
  last_pose_y = msg->y;
  last_pose_yaw = msg->yaw;

  // RCLCPP_INFO(this->get_logger(), "yaw->%f", radToDeg(yaw));
  Eigen::Quaterniond quat_eig =
  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
  Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
  Eigen::AngleAxisd(msg->yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  corrent_pose_stamped_.pose.position.x = msg->x;
  corrent_pose_stamped_.pose.position.y = msg->y;
  corrent_pose_stamped_.pose.position.z = 0.0;
  corrent_pose_stamped_.pose.orientation = quat_msg;
  double yaw_ = quaternionToYaw(corrent_pose_stamped_.pose.orientation.x, corrent_pose_stamped_.pose.orientation.y, corrent_pose_stamped_.pose.orientation.z, corrent_pose_stamped_.pose.orientation.w);
  RCLCPP_INFO(this->get_logger(), "odom x->%f y->%f yaw->%f", corrent_pose_stamped_.pose.position.x, corrent_pose_stamped_.pose.position.y, yaw_);
}

void PCLLocalization::imuReceived(sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_) {return;}

  double roll, pitch, yaw;
  tf2::Quaternion orientation;
  tf2::fromMsg(msg->orientation, orientation);
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  float acc_x = msg->linear_acceleration.x + sin(pitch) * 9.81;
  float acc_y = msg->linear_acceleration.y - cos(pitch) * sin(roll) * 9.81;
  float acc_z = msg->linear_acceleration.z - cos(pitch) * cos(roll) * 9.81;

  Eigen::Vector3f angular_velo{msg->angular_velocity.x, msg->angular_velocity.y,
    msg->angular_velocity.z};
  Eigen::Vector3f acc{acc_x, acc_y, acc_z};
  Eigen::Quaternionf quat{msg->orientation.w, msg->orientation.x, msg->orientation.y,
    msg->orientation.z};
  double imu_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);

}

void PCLLocalization::scanReceived(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // cloud.is_dense = false;  //Nanを含む可能性あるならfalse
  for(size_t i=0; i< msg->ranges.size(); ++i) {
    if(msg->ranges[i] > 30 || msg->ranges[i] < 0){msg->ranges[i] = 0;}
    cloud.points[i].x = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i) + corrent_pose_stamped_.pose.position.x;
    cloud.points[i].y = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i) + corrent_pose_stamped_.pose.position.y;
  }

  // if (use_imu_) {
  //   double received_time = msg->header.stamp.sec +
  //     msg->header.stamp.nanosec * 1e-9;
  //   lidar_undistortion_.adjustDistortion(cloud, received_time);
  // }

  pcl::PointCloud<PointType>::Ptr filtered_cloud_ptr(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>(cloud));
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);
  ndt.setInputSource(filtered_cloud_ptr);
  ndt.setInputTarget(input_elephant_cloud.makeShared());
  cloud_view(input_elephant_cloud, cloud);
  Eigen::Affine3d affine;
  tf2::fromMsg(corrent_pose_stamped_.pose, affine);
  Eigen::Vector3d trans_ = affine.translation();
  // RCLCPP_INFO(this->get_logger(), "%f %f %f %f", affine(0,0), affine(0,1), affine(0,2), affine(0,3));
  // RCLCPP_INFO(this->get_logger(), "%f %f %f %f", affine(1,0), affine(1,1), affine(1,2), affine(1,3));
  // RCLCPP_INFO(this->get_logger(), "%f %f %f %f", affine(2,0), affine(2,1), affine(2,2), affine(2,3));
  Eigen::Matrix4f init_guess = affine.matrix().cast<float>();
  // pcl::PointCloud<PointType>::Ptr output_cloud(new pcl::PointCloud<PointType>);
  rclcpp::Clock system_clock;
  time_start = chrono::system_clock::now(); // 計測開始時間
  // ndt.align(*output_cloud, init_guess);
  pcl::PointCloud<PointType> result;
  ndt.align(result);
  time_end = chrono::system_clock::now();

  RCLCPP_INFO(this->get_logger(), "time->%ld", chrono::duration_cast<chrono::milliseconds>(time_end-time_start).count());
  if (!ndt.hasConverged()) {
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    return;
  }

  Eigen::Matrix4f final_transformation = ndt.getFinalTransformation();
  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  RCLCPP_INFO(this->get_logger(), "trans x->%f y->%f", final_transformation(0.3), final_transformation(1,3));
  RCLCPP_INFO(this->get_logger(), "Iteration num->%d", ndt.getFinalNumIteration());

  corrent_pose_stamped_.header.stamp = msg->header.stamp;
  corrent_pose_stamped_.pose.position.x = static_cast<double>(final_transformation(0, 3));
  corrent_pose_stamped_.pose.position.y = static_cast<double>(final_transformation(1, 3));
  corrent_pose_stamped_.pose.position.z = static_cast<double>(final_transformation(2, 3));
  corrent_pose_stamped_.pose.orientation = quat_msg;
  pose_pub_->publish(corrent_pose_stamped_);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = msg->header.stamp;
  transform_stamped.header.frame_id = global_frame_id_;
  transform_stamped.child_frame_id = odom_frame_id_;
  transform_stamped.transform.translation.x = static_cast<double>(final_transformation(0, 3));
  transform_stamped.transform.translation.y = static_cast<double>(final_transformation(1, 3));
  transform_stamped.transform.translation.z = static_cast<double>(final_transformation(2, 3));
  transform_stamped.transform.rotation = quat_msg;
  broadcaster_.sendTransform(transform_stamped);

  path_.poses.push_back(corrent_pose_stamped_);
  path_pub_->publish(path_);

  if (enable_debug_) {
    std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
    std::cout << "has converged: " << ndt.hasConverged() << std::endl;
    std::cout << "fitness score: " << ndt.getFitnessScore() << std::endl;
    std::cout << "final transformation:" << std::endl;
    std::cout << final_transformation << std::endl;
    /* delta_angle check
     * trace(RotationMatrix) = 2(cos(theta) + 1)
     */
    double init_cos_angle = 0.5 *
      (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
    double cos_angle = 0.5 *
      (final_transformation.coeff(0,
      0) + final_transformation.coeff(1, 1) + final_transformation.coeff(2, 2) - 1);
    double init_angle = acos(init_cos_angle);
    double angle = acos(cos_angle);
    // Ref:https://twitter.com/Atsushi_twi/status/1185868416864808960
    double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));
    std::cout << "delta_angle:" << delta_angle * 180 / M_PI << "[deg]" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
  }
}

double PCLLocalization::quaternionToYaw(double x, double y, double z, double w){
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  return atan2(siny_cosp, cosy_cosp);
}

void PCLLocalization::cloud_view(pcl::PointCloud<PointType> map_cloud, pcl::PointCloud<PointType> input_cloud){
  pcl::visualization::PCLVisualizer viewer("input cloud");
  viewer.setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> elephant_color(map_cloud.makeShared(), 0, 0, 255);
  viewer.addPointCloud<PointType>(map_cloud.makeShared(), elephant_color, "map clouds");
  pcl::visualization::PointCloudColorHandlerCustom<PointType> laser_color(input_cloud.makeShared(), 0, 255, 0);
  viewer.addPointCloud<PointType>(input_cloud.makeShared(), laser_color, "input clouds");
  while(!viewer.wasStopped()){
    viewer.spinOnce(1);
  }
}


void PCLLocalization::create_elephant_map(){
  double rafter_width = 0.05;
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

  // for(size_t i=0; i<input_elephant_cloud.size(); i++){
  //   input_elephant_cloud.points[i].x -= init_pose_x;
  //   input_elephant_cloud.points[i].y -= init_pose_y;
  // }
}
