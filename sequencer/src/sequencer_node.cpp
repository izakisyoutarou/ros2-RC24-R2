#include "sequencer/sequencer_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace sequencer{

Sequencer::Sequencer(const rclcpp::NodeOptions &options) : Sequencer("", options) {}

Sequencer::Sequencer(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("sequencer_node", name_space, options)
{

}
}  // namespace sequencer