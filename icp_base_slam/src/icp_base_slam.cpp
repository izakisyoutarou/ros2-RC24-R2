#include "icp_base_slam/icp_base_slam.hpp"

IcpBaseSlam::IcpBaseSlam(const rclcpp::NodeOptions &options) : IcpBaseSlam("", options) {}

IcpBaseSlam::IcpBaseSlam(const std::string& node_name, const rclcpp::NodeOptions &options)
:  rclcpp::Node("icp_base_slam", options){
  scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
  "scan", rclcpp::SensorDataQoS(),
  bind(&IcpBaseSlam::scan_callback, this, placeholders::_1));

odom_subscriber = create_subscription<nav_msgs::msg::Odometry>(
  "odom", rclcpp::SensorDataQoS(),
  bind(&IcpBaseSlam::odom_callback, this, placeholders::_1));

odom_delay_subscriber = this->create_subscription<my_messages::msg::OdomDelay>(
  "odom_delay", 10, bind(&IcpBaseSlam::odom_delay_callback, this, placeholders::_1));
  cloud.points.resize(view_ranges/reso);
  input_elephant_cloud.points.resize(50000);
  ndt.setMaximumIterations (ndt_max_iterations_threshold);
  ndt.setResolution(ndt_resolution);   //ボクセルの辺の長さ
  ndt.setTransformationEpsilon(ndt_correspondence_distance_threshold);   //収束判定
  ndt.setStepSize (ndt_step_size);    //ニュートン法のステップサイズ
  voxel_grid_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  create_elephant_map();
  if(!use_gazebo_simulator){
    pose.x = init_pose_x;
    pose.y = init_pose_y;
  }

}

void IcpBaseSlam::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  double odom_to_lidar_length = 0.4;
  double odom_to_lidar_x = odom_to_lidar_length * cos(pose.yaw);
  double odom_to_lidar_y = odom_to_lidar_length * sin(pose.yaw);
  for(size_t i=0; i< msg->ranges.size(); ++i) {
    if(msg->ranges[i] > 30 || msg->ranges[i] < 0){msg->ranges[i] = 0;}
    cloud.points[i].x = msg->ranges[i] * cos(msg->angle_min + msg->angle_increment * i + pose.yaw) + pose.x + odom_to_lidar_x;
    cloud.points[i].y = msg->ranges[i] * sin(msg->angle_min + msg->angle_increment * i + pose.yaw) + pose.y + odom_to_lidar_y;
  }
  pcl::PointCloud<PointType>::Ptr filtered_cloud_ptr(new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType>::Ptr cloud_ptr(new pcl::PointCloud<PointType>(cloud));
  voxel_grid_filter.setInputCloud(cloud_ptr);
  voxel_grid_filter.filter(*filtered_cloud_ptr);
  ndt.setInputSource(filtered_cloud_ptr);
  ndt.setInputTarget(input_elephant_cloud.makeShared());
  pcl::PointCloud<PointType> result;
  time_start = chrono::system_clock::now();
  ndt.align(result);
  time_end = chrono::system_clock::now();

  if(!ndt.hasConverged()){
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    return;
  }

  Eigen::Matrix4d transformation_matrix = ndt.getFinalTransformation().cast<double>();

  pose.x += transformation_matrix(0,3);
  pose.y += transformation_matrix(1,3);
  pose.yaw += transformation_matrix.block<3, 3>(0, 0).eulerAngles(0,1,2)(2);

  Eigen::Vector3d poses(odom_pose.x + trans_pose.x, odom_pose.y + trans_pose.y,odom_pose.yaw + trans_pose.yaw);
  RCLCPP_INFO(this->get_logger(), "time->%ld", chrono::duration_cast<chrono::milliseconds>(time_end-time_start).count());
  RCLCPP_INFO(this->get_logger(), "pose x->%f y->%f yaw->%f°", pose.x, pose.y, radToDeg(pose.yaw));
  RCLCPP_INFO(this->get_logger(), "pose x = %f y = %f yaw = %f°", poses[0], poses[1], poses[2]);
  RCLCPP_INFO(this->get_logger(), "Iteration num->%d", ndt.getFinalNumIteration());
  RCLCPP_INFO(this->get_logger(), "dist->%f", ndt.getFitnessScore());//点群間の平均二乗距離(処理重い)
  // icp_cloud_view(input_elephant_cloud, cloud);
}

void IcpBaseSlam::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  if (!use_odom) {return;}
  double last_pose_x = msg->pose.pose.position.x;
  double last_pose_y = msg->pose.pose.position.y;
  double last_pose_yaw = quaternionToYaw(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  RCLCPP_INFO(this->get_logger(), "odom x->%f y->%f yaw->%f", last_pose_x, last_pose_y, last_pose_yaw);
  set_odom(last_pose_x, last_pose_y, last_pose_yaw);
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
  odom_pose.x=x;
  odom_pose.y=y;
  odom_pose.yaw=yaw;
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

void IcpBaseSlam::input_cloud_view(pcl::PointCloud<PointType> input_cloud){
  pcl::visualization::PCLVisualizer viewer("input cloud");
  viewer.setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color(input_cloud.makeShared(), 0, 0, 255);
  viewer.addPointCloud<PointType>(input_cloud.makeShared(), single_color, "all clouds");
  while(!viewer.wasStopped()){
    viewer.spinOnce(1);
  }
}

void IcpBaseSlam::icp_cloud_view(pcl::PointCloud<PointType> map_cloud, pcl::PointCloud<PointType> input_cloud){
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
