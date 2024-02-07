#include "sequencer/sequencer_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <fstream>


namespace sequencer{

Sequencer::Sequencer(const rclcpp::NodeOptions &options) : Sequencer("", options) {}

Sequencer::Sequencer(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("sequencer_node", name_space, options)
    
{

    _subscription_convergence = this->create_subscription<controller_interface_msg::msg::Convergence>(
        "convergence",
        _qos,
        std::bind(&Sequencer::callback_convergence, this, std::placeholders::_1)
    );

    _subscription_base_control = this->create_subscription<controller_interface_msg::msg::BaseControl>(
        "base_control",
        _qos,
        std::bind(&Sequencer::callback_base_control, this, std::placeholders::_1)
    );
    
    _subscription_is_start = this->create_subscription<std_msgs::msg::UInt8>(
        "is_start",
        _qos,
        std::bind(&Sequencer::callback_is_start, this, std::placeholders::_1)
    );

    _subscription_collection_point = this->create_subscription<std_msgs::msg::String>(
        "collection_point",
        _qos,
        std::bind(&Sequencer::callback_collection_point, this, std::placeholders::_1)
    );

    _subscription_self_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
        "self_pose",
        rclcpp::SensorDataQoS(),
        std::bind(&Sequencer::callback_self_pose, this, std::placeholders::_1)
    );

    _publisher_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
    _publisher_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    _publisher_way_point = this->create_publisher<std_msgs::msg::String>("way_point", _qos);
    _publisher_now_sequence = this->create_publisher<std_msgs::msg::String>("now_sequence", _qos);
    _publisher_move_interrupt_node = this->create_publisher<std_msgs::msg::String>("move_interrupt_node", _qos);
    

    std::ifstream ifs(ament_index_cpp::get_package_share_directory("main_executor") + "/config/spline_pid/R2_nodelist.cfg");
    std::string str;
    while(getline(ifs, str)){
        std::string token;
        std::istringstream stream(str);
        int count = 0;
        Node node;
        while(getline(stream, token, ' ')){
            if(count==0) node.name = token;
            else if(count==1) node.x = std::stold(token);
            else if(count==2) node.y = std::stold(token);
            count++;
        }
        node_list.push_back(node);
    }

    command_sequence(SEQUENCE_MODE::stop);

}

void Sequencer::callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){
    int n = 0;
    if(is_start){
        //ひし形シーケンス
        if(sequence_mode == SEQUENCE_MODE::rhombus){
            if(progress == n++){
                command_move_node("c2");
                progress++;
            }
            else if(progress == n++ && msg->spline_convergence){
                //回収コマンド
            } 
            else if(progress == n++ && msg->spline_convergence){
                //回収コマンド
            } 
        }
        //サイロシーケンス
        else if(sequence_mode == SEQUENCE_MODE::silo){
            if(progress == n++){
                command_move_node("c2");
                progress++;
            }
            else if(progress == n++ && msg->spline_convergence){
                //サイロコマンド
            }
        }
        
        //回収シーケンス
        else if(sequence_mode == SEQUENCE_MODE::collect){
            
        }
    }
}

void Sequencer::callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart) command_sequence(SEQUENCE_MODE::stop);
    is_start = false;
    progress = 0;
}

void Sequencer::callback_is_start(const std_msgs::msg::UInt8::SharedPtr msg){
    if(msg->data == 0) command_sequence(SEQUENCE_MODE::rhombus);
    else if(msg->data == 1) command_sequence(SEQUENCE_MODE::silo);
    else if(msg->data == 2) command_sequence(SEQUENCE_MODE::collect);
    is_start = true;
}

void Sequencer::callback_collection_point(const std_msgs::msg::String::SharedPtr msg){
    command_move_interrupt_node(msg->data);
}

void Sequencer::callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
    this->self_pose.x = msg->x;
    this->self_pose.y = msg->y;
    this->self_pose.z = msg->z;
    for(int i = 0; i < node_list.size(); i++){
        if(abs(node_list[i].x - self_pose.x) <= 0.1 && abs(node_list[i].y - self_pose.y) <= 0.1){
            auto msg_way_point = std::make_shared<std_msgs::msg::String>();
            msg_way_point->data = node_list[i].name;
            _publisher_way_point->publish(*msg_way_point); 
            break;
        }
    }
}

void Sequencer::command_move_node(const std::string node){
    auto msg_move_node = std::make_shared<std_msgs::msg::String>();
    msg_move_node->data = node;
    _publisher_move_node->publish(*msg_move_node);
}

void Sequencer::command_canusb(const int16_t id, const uint8_t dlc, const uint8_t data[8]){
    auto msg_canusb = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_canusb->canid = id;
    msg_canusb->candlc = dlc;
    for(int i = 0; i < dlc; i++) msg_canusb->candata[i] = data[i];
    _publisher_canusb->publish(*msg_canusb);
}

void Sequencer::command_sequence(const SEQUENCE_MODE sequence){
    sequence_mode = sequence;
    auto msg_now_sequence = std::make_shared<std_msgs::msg::String>();
    msg_now_sequence->data = sequence_list[static_cast<int>(sequence)];
    _publisher_now_sequence->publish(*msg_now_sequence);
}

void Sequencer::command_move_interrupt_node(const std::string node){
    auto msg_move_interrupt_node = std::make_shared<std_msgs::msg::String>();
    msg_move_interrupt_node->data = node;
    _publisher_move_interrupt_node->publish(*msg_move_interrupt_node);
}

}  // namespace sequencer