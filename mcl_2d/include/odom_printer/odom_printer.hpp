#include <rclcpp/rclcpp.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"

namespace odom_printer{
class OdometryPrinter : public rclcpp::Node{
public:
    explicit OdometryPrinter(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    explicit OdometryPrinter(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_odom_linear;
    rclcpp::Subscription<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _subscription_odom_angular;

    void _subscriber_callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);
    void _subscriber_callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg);

    rclcpp::QoS _qos = rclcpp::QoS(40).keep_all();

    double x,y,a;
};
}
