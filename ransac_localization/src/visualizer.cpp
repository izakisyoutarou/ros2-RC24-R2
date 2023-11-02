#include "ransac_localization/visualizer.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"
#include <string>
#include <fstream>

using namespace std;

Visualizer::Visualizer(const rclcpp::NodeOptions &options) : Visualizer("", options) {}

Visualizer::Visualizer(const std::string &name_space, const rclcpp::NodeOptions &options) : rclcpp::Node("visualizer", name_space, options),
  tf_array(get_parameter("tf_laser2robot").as_double_array()),
  odom_linear_can_id(get_parameter("canid.odom_linear").as_int()),
  odom_angular_can_id(get_parameter("canid.odom_angular").as_int()){

  rclcpp::QoS fast_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  odom_linear_subscriber = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
    "can_rx_"+ (boost::format("%x") % odom_linear_can_id).str(),fast_qos,
    bind(&Visualizer::callback_odom_linear, this, std::placeholders::_1));

  odom_angular_subscriber = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
    "can_rx_"+ (boost::format("%x") % odom_angular_can_id).str(),fast_qos,
    bind(&Visualizer::callback_odom_angular, this, std::placeholders::_1));

  self_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "self_pose", rclcpp::SensorDataQoS(), std::bind(&Visualizer::callback_selfpose, this, std::placeholders::_1));

  scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SensorDataQoS(),
  bind(&Visualizer::callback_scan, this, std::placeholders::_1));

  pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("self_localization/pose", fast_qos);

  map_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("self_localization/map", fast_qos);

  scan_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("self_localization/ransac",fast_qos);


  tf_laser2robot << tf_array[0], tf_array[1], tf_array[2], tf_array[3], tf_array[4], tf_array[5];
  vector<LineData> lines;
  load_map_config(lines);
  create_map(lines);
}

void Visualizer::callback_selfpose(const geometry_msgs::msg::Vector3::SharedPtr msg) {
  self_pose[0]=msg->x;
  self_pose[1]=msg->y;
  self_pose[2]=msg->z;
  auto pose_stamped = geometry_msgs::msg::PoseStamped();
  pose_stamped.header.stamp = this->now();
  pose_stamped.header.frame_id = "map";
  pose_stamped.pose.position.x = msg->x;
  pose_stamped.pose.position.y = msg->y;
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, msg->z);
  pose_stamped.pose.orientation.x = quaternion.x();
  pose_stamped.pose.orientation.y = quaternion.y();
  pose_stamped.pose.orientation.z = quaternion.z();
  pose_stamped.pose.orientation.w = quaternion.w();
  pose_publisher_->publish(pose_stamped);
}

void Visualizer::callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  double odom_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_odom = odom_received_time - last_odom_received_time;
  last_odom_received_time = odom_received_time;
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  const double x = (double)bytes_to_float(_candata);
  const double y = (double)bytes_to_float(_candata+4);
  diff_odom[0] = x - last_odom[0];
  diff_odom[1] = y - last_odom[1];

  if(abs(diff_odom[0]) / dt_odom > 7) diff_odom[0] = 0.0;
  if(abs(diff_odom[1]) / dt_odom > 7) diff_odom[1] = 0.0;

  odom[0] += diff_odom[0];
  odom[1] += diff_odom[1];
  last_odom[0] = x;
  last_odom[1] = y;
}

void Visualizer::callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
  double jy_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_jy = jy_received_time - last_jy_received_time;
  last_jy_received_time = jy_received_time;
  uint8_t _candata[8];
  for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
  const double yaw = (double)bytes_to_float(_candata);
  diff_odom[2] = yaw - last_odom[2];
  if(abs(diff_odom[2]) / dt_jy > 6*M_PI) diff_odom[2] = 0.0;
  odom[2] += diff_odom[2];
  last_odom[2] = yaw;
}

void Visualizer::callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
  Vector3d laser = self_pose + transform_sensor_position(tf_laser2robot);
  vector<LaserPoint> src_points = converter.scan_to_vector(msg, laser);
  sensor_msgs::msg::PointCloud2 cloud = converter.vector_to_PC2(src_points);
  scan_publisher->publish(cloud);
  map_publisher->publish(map_cloud);
}

void Visualizer::load_map_config(vector<LineData> &lines){
  ifstream ifs(ament_index_cpp::get_package_share_directory("main_executor") + "/config/ransac_localization/lines.cfg");
  string str;
  int line_count=0;
  while(getline(ifs, str)){
    string token;
    istringstream stream(str);
    int count = 0;
    LineData line_data;
    while(getline(stream, token, ' ')){   //スペース区切り
      if(line_count==0) break;
      if(count==0) line_data.p_1.x = stold(token);
      else if(count==1) line_data.p_1.y = stold(token);
      else if(count==2) line_data.p_2.x = stold(token);
      else if(count==3) line_data.p_2.y = stold(token);
      count++;
    }
    line_data.set_data();
    if(!(line_count==0)) lines.push_back(line_data);
    line_count++;
  }
  return;
}

void Visualizer::create_map(vector<LineData> &lines){
  LaserPoint map_point;
  for(size_t i=0; i<lines.size(); i++){
    if(lines[i].axis=='x'){
      for(int j=0; j<=static_cast<int>((lines[i].p_2.x - lines[i].p_1.x)*1000); j++){
        map_point.x = static_cast<double>(j)/1000 + lines[i].p_1.x;
        map_point.y = lines[i].p_1.y;
        map_points.push_back(map_point);
      }
    }
    else if(lines[i].axis=='y'){
      for(int j=0; j<=int((lines[i].p_2.y - lines[i].p_1.y)*1000); j++){
        map_point.y = static_cast<double>(j)/1000 + lines[i].p_1.y;
        map_point.x = lines[i].p_1.x;
        map_points.push_back(map_point);
      }
    }
  }
  map_cloud = converter.vector_to_PC2(map_points);
}




int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true);
  rclcpp::spin(std::make_shared<Visualizer>(options));
  rclcpp::shutdown();
  return 0;
}
