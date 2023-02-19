// #include "controller_interface/controller_interface_node.hpp"

// int main(int argc, char * argv[]){
//   rclcpp::init(argc, argv);
//   auto node1 = std::make_shared<controller_interface::ControllerInterface>();
//   auto node2 = std::make_shared<controller_interface::ControllerInterface_UDP>();

//   while (rclcpp::ok()) {
//     rclcpp::spin_some(node1);
//     rclcpp::spin_some(node2);
//   }

//   rclcpp::shutdown();
//   return 0;
// }

#include "controller_interface/controller_interface_node.hpp"
#include <thread>

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller_interface::SmartphoneGamepad>());
  rclcpp::shutdown();
  return 0;
}