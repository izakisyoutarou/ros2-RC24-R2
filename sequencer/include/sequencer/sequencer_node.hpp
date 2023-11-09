#pragma once

#include <rclcpp/rclcpp.hpp>
#include "sequencer/visibility_control.h"

namespace sequencer{

class Sequencer : public rclcpp::Node {
public:
    SEQUENCER_PUBLIC
    explicit Sequencer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    SEQUENCER_PUBLIC
    explicit Sequencer(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
   
};

}  // namespace sequencer