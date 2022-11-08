#include "icp_base_slam/icp_base_slam.hpp"

IcpBaseSlam::IcpBaseSlam(const rclcpp::NodeOptions &options) : IcpBaseSlam("", options) {}

IcpBaseSlam::IcpBaseSlam(const std::string& node_name, const rclcpp::NodeOptions &options)
:  rclcpp::Node("icp_base_slam", options){
  RCLCPP_INFO(this->get_logger(), "START");
  declare_parameter("registration_method", "NDT");
  get_parameter("registration_method", registration_method_);
  declare_parameter("voxel_leaf_size", 1.);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);

  declare_parameter("transformation_epsilon", 0.05);
  get_parameter("transformation_epsilon", transformation_epsilon_);
  declare_parameter("ndt_resolution", 1.7);
  get_parameter("ndt_resolution", ndt_resolution_);
  declare_parameter("ndt_step_size", 0.1);
  get_parameter("ndt_step_size", ndt_step_size_);

  declare_parameter("maximum_iterations", 0);
  get_parameter("maximum_iterations", maximum_iterations_);
  declare_parameter("grid_centre_x", 0.0);
  get_parameter("grid_centre_x", grid_centre_x_);
  declare_parameter("grid_centre_y", 0.0);
  get_parameter("grid_centre_y", grid_centre_y_);
  declare_parameter("grid_extent_x", 0.0);
  get_parameter("grid_extent_x", grid_extent_x_);
  declare_parameter("grid_extent_y", 0.0);
  get_parameter("grid_extent_y", grid_extent_y_);
  declare_parameter("grid_step", 0.0);
  get_parameter("grid_step", grid_step_);
  declare_parameter("optimization_step_size", 0.0);
  get_parameter("optimization_step_size", optimization_step_size_);
  declare_parameter("transformation_epsilon_2d", 0.0);
  get_parameter("transformation_epsilon_2d", transformation_epsilon_2d_);
  declare_parameter("map_voxel_leaf_size", 0.0);
  get_parameter("map_voxel_leaf_size", map_voxel_leaf_size_);


  scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "scan", rclcpp::SensorDataQoS(),
  bind(&IcpBaseSlam::scan_callback, this, placeholders::_1));

  odom_delay_subscriber = this->create_subscription<my_messages::msg::OdomDelay>(
    "odom_delay", 10, bind(&IcpBaseSlam::odom_delay_callback, this, placeholders::_1));

  simulator_odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
    "gazebo_simulator/odom",
    rclcpp::SensorDataQoS(),
    std::bind(&IcpBaseSlam::simulator_odom_callback, this, std::placeholders::_1));

  pointcloud2_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
    "filtered_cloud",rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  map_pointcloud2_publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
    "map_point_cloud",rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  cloud.points.resize(view_ranges/reso);
  input_elephant_cloud.points.resize(50000);

  if(registration_method_ == "NDT"){
    RCLCPP_INFO(this->get_logger(), "method NDT");
    ndt.setTransformationEpsilon(transformation_epsilon_);
    ndt.setResolution(ndt_resolution_);
    ndt.setStepSize (ndt_step_size_);    //ニュートン法のステップサイズ
  }
  else{
    RCLCPP_INFO(this->get_logger(), "method NDT2D");
    ndt2d.setMaximumIterations (maximum_iterations_);
    ndt2d.setGridCentre (Eigen::Vector2f (grid_centre_x_,grid_centre_y_));
    ndt2d.setGridExtent (Eigen::Vector2f (grid_extent_x_,grid_extent_y_));
    ndt2d.setGridStep (Eigen::Vector2f (grid_step_,grid_step_));
    ndt2d.setOptimizationStepSize (Eigen::Vector3d (optimization_step_size_,optimization_step_size_,0.1));
    ndt2d.setTransformationEpsilon (transformation_epsilon_2d_);
  }
  voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

  create_elephant_map();
  // map_voxel_grid_filter.setLeafSize(map_voxel_leaf_size, map_voxel_leaf_size, map_voxel_leaf_size);
  // PclCloud::Ptr map_filtered_cloud_ptr(new PclCloud());
  // PclCloud::Ptr map_cloud_ptr(new PclCloud(input_elephant_cloud));

  // voxel_grid_filter.setInputCloud(map_cloud_ptr);
  // voxel_grid_filter.filter(*map_filtered_cloud_ptr);

  if(!use_gazebo_simulator){
    pose.x = init_pose_x;
    pose.y = init_pose_y;
  }
}

void IcpBaseSlam::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  current_scan_odom = odom;
  Pose scan_odom_motion = current_scan_odom - last_scan_odom; //前回scanからのオドメトリ移動量
  Pose predict = last_estimated + scan_odom_motion;  //前回の推定値にscan間オドメトリ移動量を足し、予測位置を計算
  // RCLCPP_INFO(this->get_logger(), "current x->%f y->%f yaw->%f", current_scan_odom.x, current_scan_odom.y, current_scan_odom.yaw);

  //予測位置を基準にndtを実行
  double odom_to_lidar_length = 0.4;
  double odom_to_lidar_x = odom_to_lidar_length * cos(current_scan_odom.yaw);
  double odom_to_lidar_y = odom_to_lidar_length * sin(current_scan_odom.yaw);

  for(size_t i=0; i< msg->ranges.size(); ++i) {
    if(msg->ranges[i] > 30 || msg->ranges[i] < 0){msg->ranges[i] = 0;}
    cloud.points[i].x = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i + current_scan_odom.yaw) + current_scan_odom.x + odom_to_lidar_x;
    cloud.points[i].y = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i + current_scan_odom.yaw) + current_scan_odom.y + odom_to_lidar_y;
  }

  PclCloud::Ptr filtered_cloud_ptr(new PclCloud());
  PclCloud::Ptr cloud_ptr(new PclCloud(cloud));


  voxel_grid_filter.setInputCloud(cloud_ptr);
  voxel_grid_filter.filter(*filtered_cloud_ptr);
  PclCloud result;
  time_start = chrono::system_clock::now();
  Eigen::Matrix4d transformation_matrix;
  if(registration_method_ == "NDT"){
    ndt.setInputSource(filtered_cloud_ptr);
    ndt.align(result);
    transformation_matrix = ndt.getFinalTransformation().cast<double>();
  }
  else{
    ndt2d.setInputSource(filtered_cloud_ptr);
    ndt2d.align(result);
    transformation_matrix = ndt2d.getFinalTransformation().cast<double>();
  }
  time_end = chrono::system_clock::now();
  Pose scan_trans;
  scan_trans.x = transformation_matrix(0,3);
  scan_trans.y = transformation_matrix(1,3);
  scan_trans.yaw = transformation_matrix.block<3, 3>(0, 0).eulerAngles(0,1,2)(2);

  ndt_estimated = predict + scan_trans;
  Pose fused_pose;                       // 融合結果
  Eigen::Matrix3d fused_cov;               // センサ融合後の共分散
  double dt = 0.1;//**************odomの移動時間(scan取得時間の差)。time stampの差分ほしい*********************
  // pose_fuser->fuse_pose(ndt_estimated, scan_odom_motion, predict, fused_pose, fused_cov, dt, filtered_cloud_ptr, transformation_matrix);
  if(0<ndt.getFinalNumIteration() && registration_method_ == "NDT"){
  //   RCLCPP_INFO(this->get_logger(), "trans x->%f y->%f yaw->%f°", transformation_matrix(0,3), transformation_matrix(1,3), radToDeg(transformation_matrix.block<3, 3>(0, 0).eulerAngles(0,1,2)(2)));

  //   RCLCPP_INFO(this->get_logger(), "iteration num->%d", ndt.getFinalNumIteration());
    RCLCPP_INFO(this->get_logger(), "Max iteration num->%d", max_iteration(ndt.getFinalNumIteration()));
  //   RCLCPP_INFO(this->get_logger(), "step_size->%f", ndt.getStepSize());
  //   RCLCPP_INFO(this->get_logger(), "TransformationEpsilon->%f", ndt.getTransformationEpsilon());
  //   RCLCPP_INFO(this->get_logger(), "Max time->%d", max_time(chrono::duration_cast<chrono::milliseconds>(time_end-time_start).count()));
    // RCLCPP_INFO(this->get_logger(), "time->%d", chrono::duration_cast<chrono::milliseconds>(time_end-time_start).count());
  }
  last_scan_odom = current_scan_odom;
  last_estimated = estimated;
    // global_cloud.points.resize(filtered_cloud_ptr->points.size());
  // for(size_t i=0; i<filtered_cloud_ptr->points.size(); i++){
  //   global_cloud.points[i].x = transformation_matrix(0,0)*filtered_cloud_ptr->points[i].x + transformation_matrix(0,1)*filtered_cloud_ptr->points[i].y + transformation_matrix(0,3);
  //   global_cloud.points[i].y = transformation_matrix(1,0)*filtered_cloud_ptr->points[i].x + transformation_matrix(1,1)*filtered_cloud_ptr->points[i].y + transformation_matrix(1,3);
  // }
  // PclCloud::Ptr global_cloud_ptr(new PclCloud(global_cloud));
  pointcloud2_view(filtered_cloud_ptr, input_elephant_cloud);
}

void IcpBaseSlam::simulator_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  double yaw = quaternionToYaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  odom.x    = msg->pose.pose.position.x;
  odom.y    = msg->pose.pose.position.y;
  odom.yaw  = yaw;
  // RCLCPP_INFO(this->get_logger(), "odom x->%f y->%f yaw->%f°", pose.x, pose.y, radToDeg(pose.yaw));
}

void IcpBaseSlam::odom_delay_callback(const my_messages::msg::OdomDelay::SharedPtr msg){
  pose.x        += msg->x   - last_odom.x;
  pose.y        += msg->y   - last_odom.y;
  pose.yaw      += msg->yaw - last_odom.yaw;
  last_odom.x   =  msg->x;
  last_odom.y   =  msg->y;
  last_odom.yaw =  msg->yaw;
  // RCLCPP_INFO(this->get_logger(), "odom x->%f y->%f yaw->%f", pose.x, pose.y, pose.yaw);
}

void IcpBaseSlam::set_odom(double x, double y, double yaw){
  odom.x=x;
  odom.y=y;
  odom.yaw=yaw;
}

void IcpBaseSlam::update_data(double trans_pose_x, double trans_pose_y, double trans_pose_yaw){
  pose.x+=trans_pose_x;
  pose.y+=trans_pose_y;
  pose.yaw+=trans_pose_yaw;
}

double IcpBaseSlam::quaternionToYaw(double x, double y, double z, double w){
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  return atan2(siny_cosp, cosy_cosp);
}

void IcpBaseSlam::print4x4Matrix (const Eigen::Matrix4d & matrix){
  RCLCPP_INFO(this->get_logger(), "    | %3.6f %3.6f %3.6f |", matrix(0,0), matrix(0,1), matrix(0,2));
  RCLCPP_INFO(this->get_logger(), "R = | %3.6f %3.6f %3.6f |", matrix(1,0), matrix(1,1), matrix(1,2));
  RCLCPP_INFO(this->get_logger(), "    | %3.6f %3.6f %3.6f |", matrix(2,0), matrix(2,1), matrix(2,2));
  RCLCPP_INFO(this->get_logger(), "t = < %3.6f, %3.6f, %3.6f >", matrix(0,3), matrix(1,3), matrix(2,3));
}

void IcpBaseSlam::pointcloud2_view(PclCloud::Ptr cloud_ptr, PclCloud map_cloud){
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
    voxel_grid_filter_map.setLeafSize(map_voxel_leaf_size_, map_voxel_leaf_size_, map_voxel_leaf_size_);
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

void IcpBaseSlam::input_cloud_view(PclCloud input_cloud){
  pcl::visualization::PCLVisualizer viewer("input cloud");
  viewer.setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(input_cloud.makeShared(), 0, 0, 255);
  viewer.addPointCloud<PointType>(input_cloud.makeShared(), single_color, "all clouds");
  while(!viewer.wasStopped()){
    viewer.spinOnce(1);
  }
}

void IcpBaseSlam::icp_cloud_view(PclCloud map_cloud, PclCloud input_cloud){
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
