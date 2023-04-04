#include "injection_interface/injection_interface_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

using namespace std;
using namespace utils;

namespace injection_interface{
InjectionInterface::InjectionInterface(const rclcpp::NodeOptions &options, const int mech_num) : InjectionInterface("", options, mech_num) {}

InjectionInterface::InjectionInterface(const std::string &name_space, const rclcpp::NodeOptions &options, const int mech_num)
: rclcpp::Node("injection_interface_node", name_space, options),
mech_num(mech_num),
pole_file_path(ament_index_cpp::get_package_share_directory("injection_interface")+"/config/"+"polelist.cfg"),

tf_injection2robot(get_parameter("tf_injection2robot_m"+to_string(mech_num)).as_double_array())
{
    _subscription_pole = this->create_subscription<std_msgs::msg::String>(
        "injection_pole_m"+to_string(mech_num),
        _qos,
        std::bind(&InjectionInterface::_subscriber_callback_pole, this, std::placeholders::_1)
    );
    _subscription_self_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
        "self_pose",
        _qos,
        std::bind(&InjectionInterface::_subscriber_callback_self_pose, this, std::placeholders::_1)
    );
    _subscription_move_target_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
        "move_target_pose",
        _qos,
        std::bind(&InjectionInterface::_subscriber_callback_move_target_pose, this, std::placeholders::_1)
    );
    _subscription_is_move_tracking = this->create_subscription<std_msgs::msg::Bool>(
        "is_move_tracking",
        _qos,
        std::bind(&InjectionInterface::_subscriber_callback_is_move_tracking, this, std::placeholders::_1)
    );

    publisher_injection = this->create_publisher<injection_interface_msg::msg::InjectionCommand>("injection_command_m"+to_string(mech_num), 10);

    const auto initial_pose = this->get_parameter("initial_pose").as_double_array();
    self_pose.x = initial_pose[0];
    self_pose.y = initial_pose[1];
    self_pose.z = initial_pose[2];
}

void InjectionInterface::_subscriber_callback_pole(const std_msgs::msg::String::SharedPtr msg){
    auto injection_command = std::make_shared<injection_interface_msg::msg::InjectionCommand>();
    last_target = msg;

    geometry_msgs::msg::Vector3 robot_pose;
    TwoVector injection_pos;
    TwoVector target_pos;
    double pole_height;
    double pole_diameter;

    //ポール情報の読み込み
    ifstream ifs(pole_file_path);
    string str;
    bool target_input = false;
    while(getline(ifs, str)){
        string token;
        istringstream stream(str);
        int count = 0;
        while(getline(stream, token, ' ')){   //スペース区切り
            if(count==0 && token!=msg->data) break;
            else if(count==1) target_pos.x = stold(token);
            else if(count==2) target_pos.y = stold(token);
            else if(count==3) pole_height = stold(token);
            else if(count==4) pole_diameter = stold(token);
            target_input = true;
            count++;
        }
    }
    // RCLCPP_INFO(this->get_logger(), "x:%lf  y: %lf  直径: %lf  高さ: %lf",target_pos.x,target_pos.y,pole_diameter,pole_height);

    if(!target_input){
        RCLCPP_INFO(this->get_logger(), "射出するポール情報の入力ができませんでした");
        return;
    }
    if(is_move_tracking){
        is_correction_required = true;
        robot_pose = move_target_pose;
    }
    else{
        is_correction_required = false;
        robot_pose = self_pose;
    }

    injection_pos.x = robot_pose.x + tf_injection2robot[0] * cos(robot_pose.z);
    injection_pos.y = robot_pose.y + tf_injection2robot[1] * sin(robot_pose.z);

    TwoVector diff = target_pos - injection_pos;

    injection_command->distance = diff.length() - pole_diameter;
    injection_command->direction = diff.angle();
    injection_command->height = pole_height;
    publisher_injection->publish(*injection_command);
}

void InjectionInterface::_subscriber_callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
    self_pose.x = msg->x;
    self_pose.y = msg->y;
    self_pose.z = msg->z;
}
void InjectionInterface::_subscriber_callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg){
    is_move_tracking = msg->data;
    if(is_correction_required && !msg->data){
        _subscriber_callback_pole(last_target);
    }
}
void InjectionInterface::_subscriber_callback_move_target_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
    move_target_pose.x = msg->x;
    move_target_pose.y = msg->y;
    move_target_pose.z = msg->z;
}



}  // namespace injection_interface
