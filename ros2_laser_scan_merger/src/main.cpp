//
//   created by: Michael Jonathan (mich1342)
//   github.com/mich1342
//   24/2/2022
//

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>

#include <string>
#include <vector>
#include <array>
#include <iostream>

using namespace std;
using namespace Eigen;

class scanMerger : public rclcpp::Node
{
public:
  scanMerger() : Node("ros2_laser_scan_merger"){
    initialize_params();
    refresh_params();

    laser1_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    laser2_ = std::make_shared<sensor_msgs::msg::LaserScan>();

    auto default_qos = rclcpp::QoS(20);

    sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      topic1_, 
      default_qos, 
      std::bind(&scanMerger::scan_callback1, this, std::placeholders::_1));

    sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      topic2_, 
      default_qos, 
      std::bind(&scanMerger::scan_callback2, this, std::placeholders::_1));

    self_pose_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "self_pose",
      rclcpp::SensorDataQoS(),
      std::bind(&scanMerger::subscriber_callback_self_pose, this, std::placeholders::_1)
    );

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloudTopic_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Hello");
  }

private:
  void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg){
    time_start = chrono::system_clock::now();
    laser1_ = _msg;
    update_point_cloud_rgb();
    // cout<<"1: "<<laser1_->ranges.size()<<endl;
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
    //         _msg->ranges[100]);
    time_end = chrono::system_clock::now();
    RCLCPP_INFO(this->get_logger(), "scan time->%d[μs]", chrono::duration_cast<chrono::microseconds>(time_end-time_start).count());
  }

  void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg){
    laser2_ = _msg;
    scan_tf();
    // cout<<"2: "<<laser2_->ranges.size()<<endl;
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
    //         _msg->ranges[100]);
  }

  void subscriber_callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
    pose[0] = msg->x;
    pose[1] = msg->y;
    pose[2] = msg->z;
  }

  void scan_tf(){
    Matrix3d glob_rot;
    glob_rot << cos(pose[2]), -sin(pose[2]), 0,
                sin(pose[2]),  cos(pose[2]), 0,
                           0,             0, 1;

    tf1 = glob_rot * tf1;
    tf2 = glob_rot * tf2;

    // cout<<"hello "<<endl;
    // cout<<tf1[0]<<" "<<tf1[1]<<" "<<tf1[2]<<endl;
    // cout<<"---------------------------------------------------------------------------------"<<endl;
    // cout<<tf2[0]<<" "<<tf2[1]<<" "<<tf2[2]<<endl;
    // cout<<"---------------------------------------------------------------------------------"<<endl;
  }

  void update_point_cloud_rgb(){
    refresh_params();
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    std::vector<std::array<float, 2>> scan_data;
    int count = 0;
    float min_theta = 0;
    float max_theta = 0;
    if (show1_ && laser1_){
      float temp_min_, temp_max_;
      if( laser1_->angle_min < laser1_->angle_max){
        temp_min_ = laser1_->angle_min;
        temp_max_ = laser1_->angle_max;
      } else{
        temp_min_ = laser1_->angle_max;
        temp_max_ = laser1_->angle_min;
      }
      for (float i = temp_min_; i <= temp_max_ && count < laser1_->ranges.size(); i += laser1_->angle_increment){
        pcl::PointXYZRGB pt;

        int used_count_ = count;
        if (flip1_) used_count_ = (int)laser1_->ranges.size() - 1 - count;

        float temp_x = -1.0*laser1_->ranges[used_count_] * std::cos(i);
        float temp_y = -1.0*laser1_->ranges[used_count_] * std::sin(i);

        pt.x = temp_x * std::cos(laser1Alpha_ * M_PI / 180) - temp_y * std::sin(laser1Alpha_ * M_PI / 180) + tf1[0];
        pt.y = temp_x * std::sin(laser1Alpha_ * M_PI / 180) + temp_y * std::cos(laser1Alpha_ * M_PI / 180) + tf1[1];
        pt.z = laser1ZOff_;

        if ((i < (laser1AngleMin_ * M_PI / 180)) || (i > (laser1AngleMax_ * M_PI / 180))){
          if (inverse1_){
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta) min_theta = theta_;
            if (theta_ > max_theta) max_theta = theta_;
          }
        }
        else{
          if (!inverse1_){
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta) min_theta = theta_;
            if (theta_ > max_theta) max_theta = theta_;
          }
        }
        count++;
      }
    }

    count = 0;
    if (show2_ && laser2_){
      float temp_min_, temp_max_;
      if( laser2_->angle_min < laser2_->angle_max){
        temp_min_ = laser2_->angle_min;
        temp_max_ = laser2_->angle_max;
      } else{
        temp_min_ = laser2_->angle_max;
        temp_max_ = laser2_->angle_min;
      }
      for (float i = temp_min_; i <= temp_max_ && count < laser2_->ranges.size(); i += laser2_->angle_increment){
        pcl::PointXYZRGB pt;
        int used_count_ = count;
        if (flip2_) used_count_ = (int)laser2_->ranges.size() - 1 - count;

        float temp_x = -1.0*laser2_->ranges[used_count_] * std::cos(i);
        float temp_y = -1.0*laser2_->ranges[used_count_] * std::sin(i);

        pt.x = temp_x * std::cos(laser2Alpha_ * M_PI / 180) - temp_y * std::sin(laser2Alpha_ * M_PI / 180) + tf2[0];
        pt.y = temp_x * std::sin(laser2Alpha_ * M_PI / 180) + temp_y * std::cos(laser2Alpha_ * M_PI / 180) + tf2[1];
        pt.z = laser2ZOff_;

        if ((i < (laser2AngleMin_ * M_PI / 180)) || (i > (laser2AngleMax_ * M_PI / 180))){
          if (inverse2_){
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta) min_theta = theta_;
            if (theta_ > max_theta) max_theta = theta_;
          }
        }
        else{
          if (!inverse2_){
            cloud_.points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta) min_theta = theta_;
            if (theta_ > max_theta) max_theta = theta_;
          }
        }
        count++;
      }
    }

    auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

    pclToROSMsg(cloud_, pc2_msg_);
    pc2_msg_->header.frame_id = cloudFrameId_;
    pc2_msg_->header.stamp = now();
    pc2_msg_->is_dense = false;
    // cout<<"cloud_pub"<<pc2_msg_->data.size()<<endl;
    point_cloud_pub_->publish(*pc2_msg_);
  }

  float GET_R(float x, float y){ return sqrt(x * x + y * y); }
  float GET_THETA(float x, float y){
    float temp_res;
    if ((x != 0)){
      temp_res = atan(y / x);
    }else {
      if (y >= 0){
        temp_res = M_PI / 2;
      }else {
        temp_res = -M_PI / 2;
      }
    }
    if (temp_res > 0){
      if (y < 0){
        temp_res -= M_PI;
      }
    }
    else if (temp_res < 0){
      if (x < 0){
        temp_res += M_PI;
      }
    }
    // RCLCPP_INFO(this->get_logger(), "x: '%f', y: '%f', a: '%f'", x, y, temp_res);

    return temp_res;
  }

  float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle){
    return (magnitude_1 + current_angle * ((magnitude_2 - magnitude_1) / (angle_2 - angle_1)));
  }

  void pclToROSMsg(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, sensor_msgs::msg::PointCloud2::SharedPtr& pc2_msg){
    pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    
    pc2_msg->header.frame_id = cloudFrameId_;
    pc2_msg->header.stamp = now();
    pc2_msg->height = 1;
    pc2_msg->width = cloud.points.size();
    pc2_msg->is_bigendian = false;
    pc2_msg->is_dense = false;
    
    sensor_msgs::PointCloud2Modifier modifier(*pc2_msg);
    modifier.setPointCloud2Fields(3, "x",   1, sensor_msgs::msg::PointField::FLOAT32,
                                     "y",   1, sensor_msgs::msg::PointField::FLOAT32,
                                     "z",   1, sensor_msgs::msg::PointField::FLOAT32);
    
    sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2_msg, "z");

    for (const auto& point : cloud.points) {
        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
        
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
  }

  void initialize_params(){
    this->declare_parameter("pointCloudTopic", "scan_cloud");
    this->declare_parameter("pointCloutFrameId", "laser");

    this->declare_parameter("scanTopic1", "scan_1st");
    this->declare_parameter("laser1XOff", 0.0);
    this->declare_parameter("laser1YOff", 0.0);
    this->declare_parameter("laser1ZOff", 0.0);
    this->declare_parameter("laser1Alpha", 0.0);
    this->declare_parameter("laser1AngleMin", -180.0);
    this->declare_parameter("laser1AngleMax", 180.0);
    this->declare_parameter("show1", true);
    this->declare_parameter("flip1", true);
    this->declare_parameter("inverse1", false);

    this->declare_parameter("scanTopic2", "scan_2nd");
    this->declare_parameter("laser2XOff", 0.0);
    this->declare_parameter("laser2YOff", 0.0);
    this->declare_parameter("laser2ZOff", 0.0);
    this->declare_parameter("laser2Alpha", 225.0);
    this->declare_parameter("laser2AngleMin", -180.0);
    this->declare_parameter("laser2AngleMax", 180.0);
    this->declare_parameter("show2", true);
    this->declare_parameter("flip2", true);
    this->declare_parameter("inverse2", false);
  }
  
  void refresh_params(){
    this->get_parameter_or<std::string>("pointCloudTopic", cloudTopic_, "scan_cloud");
    this->get_parameter_or<std::string>("pointCloutFrameId", cloudFrameId_, "laser");
    this->get_parameter_or<std::string>("scanTopic1", topic1_, "scan_1st");
    this->get_parameter_or<float>("laser1XOff", laser1XOff_, 0.0);
    this->get_parameter_or<float>("laser1YOff", laser1YOff_, 0.0);
    this->get_parameter_or<float>("laser1ZOff", laser1ZOff_, 0.0);
    this->get_parameter_or<float>("laser1Alpha", laser1Alpha_, 0.0);
    this->get_parameter_or<float>("laser1AngleMin", laser1AngleMin_, -180.0);
    this->get_parameter_or<float>("laser1AngleMax", laser1AngleMax_, 180.0);
    this->get_parameter_or<bool>("show1", show1_, true);
    this->get_parameter_or<bool>("flip1", flip1_, true);
    this->get_parameter_or<bool>("inverse1", inverse1_, false);
    this->get_parameter_or<std::string>("scanTopic2", topic2_, "scan_2nd");
    this->get_parameter_or<float>("laser2XOff", laser2XOff_, 0.0);
    this->get_parameter_or<float>("laser2YOff", laser2YOff_, 0.0);
    this->get_parameter_or<float>("laser2ZOff", laser2ZOff_, 0.0);
    this->get_parameter_or<float>("laser2Alpha", laser2Alpha_, 0.0);
    this->get_parameter_or<float>("laser2AngleMin", laser2AngleMin_, -180.0);
    this->get_parameter_or<float>("laser2AngleMax", laser2AngleMax_, 180.0);
    this->get_parameter_or<bool>("show2", show2_, true);
    this->get_parameter_or<bool>("flip2", flip2_, true);
    this->get_parameter_or<bool>("inverse2", inverse2_, false);

    tf1[0] = laser1XOff_;
    tf1[1] = laser1YOff_;
    tf1[2] = laser1ZOff_;

    tf2[0] = laser2XOff_;
    tf2[1] = laser2YOff_;
    tf2[2] = laser2ZOff_;
  }

  std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
  bool show1_, show2_, flip1_, flip2_, inverse1_, inverse2_;
  float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
  uint8_t laser1R_, laser1G_, laser1B_;
  Vector3d pose, tf1, tf2;

  float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr self_pose_;

  sensor_msgs::msg::LaserScan::SharedPtr laser1_;
  sensor_msgs::msg::LaserScan::SharedPtr laser2_;
  chrono::system_clock::time_point time_start, time_end, rotation_correction_time_start;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scanMerger>());
  rclcpp::shutdown();
  return 0;
}