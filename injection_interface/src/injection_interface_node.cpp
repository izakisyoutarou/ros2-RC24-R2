#include "injection_interface/injection_interface_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

using namespace std;
using namespace utils;

namespace injection_interface{
    InjectionInterface::InjectionInterface(const rclcpp::NodeOptions &options) : InjectionInterface("", options) {}
    InjectionInterface::InjectionInterface(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("injection_interface_node", name_space, options),
        tf_injection2robot(get_parameter("tf_injection2robot").as_double_array()),
        strage_backside(get_parameter("strage_backside").as_double_array()),
        strage_front(get_parameter("strage_front").as_double_array())
        {
            _sub_aim_in_storage = this->create_subscription<std_msgs::msg::Bool>(
                "is_backside",
                _qos,
                std::bind(&InjectionInterface::_callback_aim_in_storage, this, std::placeholders::_1)
            );

            _sub_is_move_tracking = this->create_subscription<std_msgs::msg::Bool>(
                "is_move_tracking",
                _qos,
                std::bind(&InjectionInterface::_callback_is_move_tracking, this, std::placeholders::_1)
            );

            _sub_self_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
                "self_pose",
                _qos,
                std::bind(&InjectionInterface::_callback_self_pose, this, std::placeholders::_1)
            );

            _sub_move_target_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
                "move_target_pose",
                _qos,
                std::bind(&InjectionInterface::_callback_move_target_pose, this, std::placeholders::_1)
            );

            _pub_injection = this->create_publisher<injection_interface_msg::msg::InjectionCommand>("injection_command", 10);

            const auto initial_pose = this->get_parameter("initial_pose").as_double_array();
            self_pose.x = initial_pose[0];
            self_pose.y = initial_pose[1];
            self_pose.z = initial_pose[2];
        }

        void InjectionInterface::_callback_aim_in_storage(const std_msgs::msg::Bool::SharedPtr msg){
            auto injection_command = std::make_shared<injection_interface_msg::msg::InjectionCommand>();
            last_target = msg;

            geometry_msgs::msg::Vector3 robot_pose;
            TwoVector injection_pos;
            TwoVector target_pos;
            double target_height;
            bool target_input = false;

            // RCLCPP_INFO(this->get_logger(), "x:%lf  y: %lf  直径: %lf  高さ: %lf",target_pos.x,target_pos.y,pole_diameter,pole_height);

            if(msg->data){
                target_pos.x = strage_backside[0];
                target_pos.y = strage_backside[1];
                target_height = strage_backside[2];
                target_input = true;
            }
            else{
                target_pos.x = strage_front[0];
                target_pos.y = strage_front[1];
                target_height = strage_front[2];
                target_input = true;
            }

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

            //ロボットの本体座標と射出機構のずれを補正した数字
            injection_pos.x = robot_pose.x + tf_injection2robot[0]*cos(robot_pose.z) - tf_injection2robot[1]*sin(robot_pose.z);
            injection_pos.y = robot_pose.y + tf_injection2robot[0]*sin(robot_pose.z) + tf_injection2robot[1]*cos(robot_pose.z);

            RCLCPP_INFO(this->get_logger(), "射出位置 x:%lf y:%lf", injection_pos.x, injection_pos.y);

            TwoVector diff = target_pos - injection_pos;

            injection_command->distance = diff.length();
            // injection_command->direction = diff.angle() - self_pose.z;
            injection_command->height = target_height;

            _pub_injection->publish(*injection_command);
        }

        void InjectionInterface::_callback_is_move_tracking(const std_msgs::msg::Bool::SharedPtr msg){
            is_move_tracking = msg->data;
            
            //足回り追従が終わっており、
            if(is_correction_required && !msg->data){
                _callback_aim_in_storage(last_target);
            }
        }

        void InjectionInterface::_callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
            self_pose.x = msg->x;
            self_pose.y = msg->y;
            self_pose.z = msg->z;
        }

        void InjectionInterface::_callback_move_target_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
            move_target_pose.x = msg->x;
            move_target_pose.y = msg->y;
            move_target_pose.z = msg->z;
        }
}  // namespace injection_interface
