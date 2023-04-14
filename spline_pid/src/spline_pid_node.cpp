#include "spline_pid/spline_pid_node.hpp"
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include <float.h>
#include <cmath>

using namespace utils;

namespace spline_pid{

SplinePid::SplinePid(const rclcpp::NodeOptions &options) : SplinePid("", options) {}

SplinePid::SplinePid(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("spline_pid_node", name_space, options),
limit_linear(DBL_MAX,
get_parameter("linear_max_vel").as_double(),
get_parameter("linear_max_acc").as_double(),
get_parameter("linear_max_dec").as_double() ),
limit_angular(DBL_MAX,
dtor(get_parameter("angular_max_vel").as_double()),
dtor(get_parameter("angular_max_acc").as_double()),
dtor(get_parameter("angular_max_dec").as_double()) ),

curvature_attenuation_rate(get_parameter("curvature_attenuation_rate").as_double()),
linear_planner_vel_limit_gain(get_parameter("linear_planner_vel_limit_gain").as_double()),

linear_planner_gain(get_parameter("linear_planner_gain").as_double()),
linear_pos_gain(get_parameter("linear_pos_gain").as_double()),
linear_pos_integral_gain(get_parameter("linear_pos_integral_gain").as_double()),

angular_planner_gain(get_parameter("angular_planner_gain").as_double()),
angular_pos_gain(get_parameter("angular_pos_gain").as_double()),
angular_pos_integral_gain(get_parameter("angular_pos_integral_gain").as_double()),

linear_pos_tolerance(get_parameter("linear_pos_tolerance").as_double()),
angular_pos_tolerance(dtor(get_parameter("angular_pos_tolerance").as_double()))
{
    const auto interval_ms = this->get_parameter("interval_ms").as_int();
    sampling_time = interval_ms / 1000.0;
    const auto initial_pose = this->get_parameter("initial_pose").as_double_array();


    _subscription_path = this->create_subscription<path_msg::msg::Path>(
        "spline_path",
        _qos,
        std::bind(&SplinePid::_subscriber_callback_path, this, std::placeholders::_1)
    );
    _subscription_base_control = this->create_subscription<controller_interface_msg::msg::BaseControl>(
        "pub_base_control",
        _qos,
        std::bind(&SplinePid::_subscriber_callback_base_control, this, std::placeholders::_1)
    );
    _subscription_self_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
        "self_pose",
        _qos,
        std::bind(&SplinePid::_subscriber_callback_self_pose, this, std::placeholders::_1)
    );
    _subscription_target_angle = this->create_subscription<geometry_msgs::msg::Vector3>(
        "move_target_angle_diff",
        _qos,
        std::bind(&SplinePid::_subscriber_callback_target_angle_diff, this, std::placeholders::_1)
    );
    _pub_timer = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),
        [this] { _publisher_callback(); }
    );

    publisher_velocity = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);
    publisher_is_tracking = this->create_publisher<std_msgs::msg::Bool>("is_move_tracking", _qos);
    publisher_target_pose = this->create_publisher<geometry_msgs::msg::Vector3>("move_target_pose", _qos);
    publisher_linear = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    publisher_angular = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

    velPlanner_linear.limit(limit_linear);
    velPlanner_angular.limit(limit_angular);

    self_pose.x = initial_pose[0];
    self_pose.y = initial_pose[1];
    self_pose.z = initial_pose[2];
}

void SplinePid::_publisher_callback(){
    auto cmd_velocity = std::make_shared<geometry_msgs::msg::Twist>();
    auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_linear->canid = 0x110;
    msg_linear->candlc = 8;
    auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_angular->canid = 0x111;
    msg_angular->candlc = 4;

    if(max_trajectories>0){ //追従時

    // 曲率考慮
    // VelPlannerLimit current_limit_linear = limit_linear;
    // current_limit_linear.vel = current_limit_linear.vel - (current_limit_linear.vel * std::abs(path->curvature.at(current_count)) * curvature_attenuation_rate);
    // current_limit_linear.vel *= linear_planner_vel_limit_gain;
    // velPlanner_linear.limit(current_limit_linear);

    // velPlanner_linear.pos(path->length.back(), velPlanner_linear.vel());

    velPlanner_linear.cycle();
    velPlanner_angular.cycle();

    // RCLCPP_INFO(this->get_logger(), "target:%lf  vel:%lf", path->length.back(), velPlanner_linear.vel());

    bool is_target_changed = false;
    if(!is_target_arrived){
    while(path->length.at(current_count) < velPlanner_linear.pos()){
        current_count++;
        is_target_changed = true;
        if(!(current_count+1 < max_trajectories)){
            RCLCPP_INFO(this->get_logger(), "目標が終点に到達しました");
            is_target_arrived = true;
            break;
        }
    }
    }
    if(is_target_changed){
        x_diff = path->x.at(current_count) - last_target_position.x;
        y_diff = path->y.at(current_count) - last_target_position.y;
    }

    const double error_x = path->x.at(current_count) - self_pose.x;
    error_x_integral += error_x * sampling_time;
    const double error_y = path->y.at(current_count) - self_pose.y;
    error_y_integral += error_y * sampling_time;
    const double error_a = velPlanner_angular.pos() - self_pose.z;
    error_a_integral += error_a * sampling_time;

    //並進速度処理
    cmd_velocity->linear.x = velPlanner_linear.vel() * (x_diff/(std::abs(x_diff)+std::abs(y_diff)))*linear_planner_gain + error_x*linear_pos_gain + error_x_integral*linear_pos_integral_gain;
    cmd_velocity->linear.y = velPlanner_linear.vel() * (y_diff/(std::abs(x_diff)+std::abs(y_diff)))*linear_planner_gain + error_y*linear_pos_gain + error_y_integral*linear_pos_integral_gain;
    const double vel_length = std::sqrt(cmd_velocity->linear.x*cmd_velocity->linear.x + cmd_velocity->linear.y*cmd_velocity->linear.y);
    if(vel_length > limit_linear.vel){
        cmd_velocity->linear.x *= limit_linear.vel / vel_length;
        cmd_velocity->linear.y *= limit_linear.vel / vel_length;
    }
    //回転速度処理
    cmd_velocity->angular.z = velPlanner_angular.vel()*angular_planner_gain + error_a*angular_pos_gain + error_a_integral*angular_pos_integral_gain;
    // RCLCPP_INFO(this->get_logger(), "angle vel %lf",velPlanner_angular.vel());
    // RCLCPP_INFO(this->get_logger(), "angle pos %lf",velPlanner_angular.pos());
    cmd_velocity->angular.z = constrain(cmd_velocity->angular.z, -limit_angular.vel, limit_angular.vel);

    //収束判断
    const double linear_error_length = std::sqrt(error_x*error_x + error_y*error_y);
    const double angular_error_length = std::abs(error_a);
    bool is_linear_arrived = false;
    bool is_angular_arrived = false;
    if(is_target_arrived && linear_error_length <= linear_pos_tolerance){
        cmd_velocity->linear.x = 0.0;
        cmd_velocity->linear.y = 0.0;
        is_linear_arrived = true;
    }
    if(is_target_arrived && angular_error_length <= angular_pos_tolerance){
        cmd_velocity->angular.z = 0.0;
        is_angular_arrived = true;
    }
    if(is_linear_arrived && is_angular_arrived && !is_arrived){
        is_arrived = true;
        RCLCPP_INFO(this->get_logger(), "状態が目標に到達しました");
        // 追従終了の出版
        publish_is_tracking(false);
    }
    else if((!is_linear_arrived || !is_angular_arrived) &&is_arrived){
        is_arrived = false;
        RCLCPP_INFO(this->get_logger(), "状態が目標から外れました");
        publish_is_tracking(true);
    }

    //CANメッセージのパック
    uint8_t _candata[8];
    float_to_bytes(_candata, static_cast<float>(cmd_velocity->linear.x));
    float_to_bytes(_candata+4, static_cast<float>(cmd_velocity->linear.y));
    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata[i];

    float_to_bytes(_candata, static_cast<float>(cmd_velocity->angular.z));
    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata[i];


    //前回値の更新
    last_target_position.x = path->x.at(current_count);
    last_target_position.y = path->y.at(current_count);
    last_target_position.z = path->angle.at(current_count);

    }

    //出版
    publisher_velocity->publish(*cmd_velocity); //シミュレータ用コマンド

    publisher_linear->publish(*msg_linear);
    publisher_angular->publish(*msg_angular);
}

void SplinePid::_subscriber_callback_path(const path_msg::msg::Path::SharedPtr msg){
    const int size = msg->length.size();
    // RCLCPP_INFO(this->get_logger(),"x:%d y:%d a:%d len:%d cur:%d\n",msg->x.size(),msg->y.size(),msg->angle.size(),msg->length.size(),msg->curvature.size());
    if(size==msg->x.size() && size==msg->y.size() && size==msg->angle.size() && size==msg->curvature.size()){
        path = msg;
        // 等加速度モードから抜け出すために最初に現在状態更新を行う
        velPlanner_linear.current(0.0, velPlanner_linear.vel(), velPlanner_linear.acc());
        velPlanner_angular.current(self_pose.z, velPlanner_angular.vel(), velPlanner_angular.acc());

        //速度計画の指令
        VelPlannerLimit current_limit_linear = limit_linear;
        current_limit_linear.vel *= linear_planner_vel_limit_gain;
        velPlanner_linear.limit(current_limit_linear);

        velPlanner_linear.pos(path->length.back(), velPlanner_linear.vel());
        velPlanner_angular.pos(path->angle.back(),velPlanner_angular.vel());

        current_count = 0;
        last_target_position.x = path->x.front();
        last_target_position.y = path->y.front();
        last_target_position.z = self_pose.z;
        is_arrived = false;
        is_target_arrived = false;
        max_trajectories = size;

        // 追従開始の出版
        publish_is_tracking(true);

        //目標姿勢の出版
        auto msg_target_pose =  std::make_shared<geometry_msgs::msg::Vector3>();
        msg_target_pose->x = path->x.back();
        msg_target_pose->y = path->y.back();
        msg_target_pose->z = path->angle.back();
        publisher_target_pose->publish(*msg_target_pose);
    }
    else {
        RCLCPP_INFO(this->get_logger(), "軌道点のサイズが等しくありません");
    }
}

void SplinePid::_subscriber_callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart){
        max_trajectories = 0;
        RCLCPP_INFO(this->get_logger(), "経路追従を停止しました");
    }
}

void SplinePid::_subscriber_callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
    this->self_pose.x = msg->x;
    this->self_pose.y = msg->y;
    this->self_pose.z = msg->z;
}
void SplinePid::_subscriber_callback_target_angle_diff(const geometry_msgs::msg::Vector3::SharedPtr msg){
    if(max_trajectories>0){
    velPlanner_angular.pos(msg->z,velPlanner_angular.vel() + self_pose.z);
    }
}

void SplinePid::publish_is_tracking(const bool is_tracking){
    auto msg_is_tracking = std::make_shared<std_msgs::msg::Bool>();
    msg_is_tracking->data = is_tracking;
    publisher_is_tracking->publish(*msg_is_tracking);
}

}  // namespace spline_pid
