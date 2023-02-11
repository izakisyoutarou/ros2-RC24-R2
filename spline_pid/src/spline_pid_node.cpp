#include "spline_pid/spline_pid_node.hpp"
#include "utilities/can_utils.hpp"
#include <float.h>
#include <cmath>

namespace spline_pid{

SplinePid::SplinePid(const rclcpp::NodeOptions &options) : SplinePid("", options) {}

SplinePid::SplinePid(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("spline_pid_node", name_space, options), limit_linear(DBL_MAX,0.0,0.0,0.0), limit_angular(DBL_MAX,0.0,0.0,0.0){

    // declare_parameter("interval_ms", 1);
    auto interval_ms = this->get_parameter("interval_ms").as_int();
    auto initial_pose = this->get_parameter("initial_pose").as_double_array();
    curvature_attenuation_rate = this->get_parameter("curvature_attenuation_rate").as_double();
    linear_pos_gain = this->get_parameter("linear_pos_gain").as_double();
    angular_pos_gain = this->get_parameter("angular_pos_gain").as_double();

    limit_linear.vel = this->get_parameter("linear_max_vel").as_double();
    limit_linear.acc = this->get_parameter("linear_max_acc").as_double();
    limit_linear.dec = this->get_parameter("linear_max_dec").as_double();
    limit_angular.vel = this->get_parameter("angular_max_vel").as_double();
    limit_angular.acc = this->get_parameter("angular_max_acc").as_double();
    limit_angular.dec = this->get_parameter("angular_max_dec").as_double();

    _subscription_path = this->create_subscription<path_msg::msg::Path>(
        "spline_path",
        _qos,
        std::bind(&SplinePid::_subscriber_callback_path, this, std::placeholders::_1)
    );
    _subscription_self_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
        "self_pose",
        _qos,
        std::bind(&SplinePid::_subscriber_callback_self_pose, this, std::placeholders::_1)
    );
    _pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this] { _publisher_callback(); }
    );

    publisher_velocity = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);
    publisher_linear = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    publisher_angular = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

    velPlanner_linear.limit(limit_linear);
    velPlanner_linear.current(0.0, 0.0, 0.0);
    velPlanner_angular.limit(limit_angular);
    velPlanner_angular.current(initial_pose[2], 0.0, 0.0);
}

void SplinePid::_publisher_callback(){
    const int max_trajectories = path->x.size();

    auto cmd_velocity = std::make_shared<geometry_msgs::msg::Twist>();
    auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_linear->canid = 0x110;
    msg_linear->candlc = 8;
    auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_angular->canid = 0x111;
    msg_angular->candlc = 4;

    VelPlannerLimit current_limit_linear = limit_linear;
    current_limit_linear.vel = current_limit_linear.vel - (current_limit_linear.vel * std::abs(path->curvature.at(current_count)) * curvature_attenuation_rate);
    velPlanner_linear.limit(current_limit_linear);

    velPlanner_linear.cycle();
    velPlanner_angular.cycle();

    bool is_target_changed = false;
    if(is_arrived){
    while(path->length.at(current_count) < velPlanner_linear.pos()){
        current_count++;
        is_target_changed = true;
        if(!(current_count < max_trajectories-1)){
            RCLCPP_INFO(this->get_logger(), "終点に到達しました");
            is_arrived = true;
            break;
        }
    }
    }
    if(is_target_changed){
        x_diff = path->x.at(current_count) - last_target_position.x;
        y_diff = path->y.at(current_count) - last_target_position.y;
    }
    const double error_x = path->x.at(current_count) - self_pose->x;
    const double error_y = path->y.at(current_count) - self_pose->y;
    const double error_a = path->angle.at(current_count) - self_pose->z;

    cmd_velocity->linear.x = velPlanner_linear.vel() * (x_diff/(x_diff+y_diff)) + error_x*linear_pos_gain;
    cmd_velocity->linear.y = velPlanner_linear.vel() * (y_diff/(x_diff+y_diff)) + error_y*linear_pos_gain;
    cmd_velocity->angular.z = velPlanner_angular.vel() + error_a*angular_pos_gain;

    uint8_t _candata[8];
    float_to_bytes(_candata, static_cast<float>(cmd_velocity->linear.x));
    float_to_bytes(_candata+4, static_cast<float>(cmd_velocity->linear.y));
    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata[i];

    float_to_bytes(_candata, static_cast<float>(cmd_velocity->angular.z));
    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata[i];

    publisher_velocity->publish(*cmd_velocity); //出版
    publisher_linear->publish(*msg_linear);
    publisher_angular->publish(*msg_angular);

    last_target_position.x = path->x.at(current_count);
    last_target_position.y = path->y.at(current_count);
}

void SplinePid::_subscriber_callback_path(const path_msg::msg::Path::SharedPtr msg){
    const int size = msg->x.size();
    if(size==msg->y.size() && size==msg->angle.size() && size==msg->length.size() && size==msg->curvature.size()){
        path = msg.get();
        velPlanner_linear.pos(msg->length.back(), velPlanner_linear.vel());
        current_count = 0;
        last_target_position.x = msg->x.at(0);
        last_target_position.y = msg->y.at(0);
        is_arrived = false;
    }
    else {
        RCLCPP_INFO(this->get_logger(), "軌道点のサイズが等しくありません");
    }
}

void SplinePid::_subscriber_callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
    this->self_pose = msg.get();
}

}  // namespace spline_pid
