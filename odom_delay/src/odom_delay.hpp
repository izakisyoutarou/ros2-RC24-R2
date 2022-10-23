#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include "my_messages/msg/odom_delay.hpp"

using namespace std::chrono_literals;

class OdomDelayPublisher : public rclcpp::Node{
public:
  OdomDelayPublisher();
private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void _subscriber_callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  double quaternionToYaw(double x, double y, double z, double w);
  double degToRad(double degree){return degree*M_PI/180;}
  double radToDeg(double rad){return rad*180/M_PI;}
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _subscription_odom;
  rclcpp::Publisher<my_messages::msg::OdomDelay>::SharedPtr odom_delay_publisher;
  geometry_msgs::msg::PoseStamped corrent_pose_stamped_;
  double last_odom_received_time;
  double drift_rate = 1.0;
};
