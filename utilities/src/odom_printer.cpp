#include "odom_printer.hpp"
#include "utilities/can_utils.hpp"
#include "rclcpp/time.hpp"

namespace odom_printer{
    OdometryPrinter::OdometryPrinter(const rclcpp::NodeOptions &options) : OdometryPrinter("", options) {}

    OdometryPrinter::OdometryPrinter(const std::string &name_space, const rclcpp::NodeOptions &options)
    : rclcpp::Node("odom_printer_node", name_space, options) {

        _subscription_odom_linear = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_110",
                _qos,
                std::bind(&OdometryPrinter::_subscriber_callback_odom_linear, this, std::placeholders::_1)
        );
        _subscription_odom_angular = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_111",
                _qos,
                std::bind(&OdometryPrinter::_subscriber_callback_odom_angular, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Print if the topic is output !");
    }

    void OdometryPrinter::_subscriber_callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
        uint8_t _candata[8];
        for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
        x = (double)bytes_to_float(_candata);
        y = (double)bytes_to_float(_candata+4);
    }

    void OdometryPrinter::_subscriber_callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
        uint8_t _candata[8];
        for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
        double a = (double)bytes_to_float(_candata);

        RCLCPP_INFO(this->get_logger(), "ODM   x:%lf  y:%lf  ang:%lf", x,y,a);
    }

}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odom_printer::OdometryPrinter>());
  rclcpp::shutdown();
  return 0;
}
