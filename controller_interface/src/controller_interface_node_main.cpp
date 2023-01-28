#include "controller_interface/controller_interface_node.hpp"

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller_interface::ControllerInterface>());
  rclcpp::shutdown();
  return 0;
}