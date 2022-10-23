#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using std::placeholders::_1;

class MySubscriber : public rclcpp::Node{
public:
  MySubscriber() : Node("my_subscriber"){
    geometry_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "pcl_pose", rclcpp::SystemDefaultsQoS(),
    std::bind(&MySubscriber::geometry_callback, this, _1));
  }

private:
  void geometry_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(), "pcl_localization_pose x->%f y->%f", msg->pose.position.x, msg->pose.position.y);
  }
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::ConstSharedPtr geometry_subscriber;

};




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MySubscriber>());
  rclcpp::shutdown();
  return 0;
}
