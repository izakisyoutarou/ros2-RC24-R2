#include "odom_delay.hpp"

using namespace std::chrono_literals;
OdomDelayPublisher::OdomDelayPublisher() : Node("odom_delay"){
  odom_subscriber = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", rclcpp::SensorDataQoS(),
    bind(&OdomDelayPublisher  ::odom_callback, this, std::placeholders::_1));
  odom_delay_publisher = this->create_publisher<my_messages::msg::OdomDelay>("odom_delay", 10);

  _subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
    "gazebo_simulator/odom",
    rclcpp::SensorDataQoS(),
    std::bind(&OdomDelayPublisher::_subscriber_callback_odom, this, std::placeholders::_1)
  );
}


void OdomDelayPublisher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  auto odom_delay_pose = my_messages::msg::OdomDelay();
  double current_odom_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  double dt_odom = current_odom_received_time - last_odom_received_time;
  last_odom_received_time = current_odom_received_time;
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

  Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  Eigen::Vector3d odom{
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z};
  Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom * drift_rate;

  corrent_pose_stamped_.pose.position.x += delta_position.x();
  corrent_pose_stamped_.pose.position.y += delta_position.y();
  corrent_pose_stamped_.pose.orientation = quat_msg;
  odom_delay_pose.x = corrent_pose_stamped_.pose.position.x;
  odom_delay_pose.y = corrent_pose_stamped_.pose.position.y;
  odom_delay_pose.yaw = quaternionToYaw(corrent_pose_stamped_.pose.orientation.x,
                                        corrent_pose_stamped_.pose.orientation.y,
                                        corrent_pose_stamped_.pose.orientation.z,
                                        corrent_pose_stamped_.pose.orientation.w);

  RCLCPP_INFO(this->get_logger(), "odom x->%f y->%f yaw->%f°", odom_delay_pose.x, odom_delay_pose.y, radToDeg(odom_delay_pose.yaw));
  odom_delay_publisher->publish(odom_delay_pose);
}

void OdomDelayPublisher::_subscriber_callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
  auto odom_delay_pose = my_messages::msg::OdomDelay();
  odom_delay_pose.x = msg->pose.pose.position.x;
  odom_delay_pose.y = msg->pose.pose.position.y;
  odom_delay_pose.yaw = quaternionToYaw(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  RCLCPP_INFO(this->get_logger(), "odom x->%f y->%f yaw->%f°", msg->pose.pose.position.x, msg->pose.pose.position.y, radToDeg(odom_delay_pose.yaw));
  odom_delay_publisher->publish(odom_delay_pose);
}

double OdomDelayPublisher::quaternionToYaw(double x, double y, double z, double w){
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  return atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomDelayPublisher>());
  rclcpp::shutdown();
  return 0;
}
