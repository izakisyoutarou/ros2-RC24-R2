#include "sequencer/sequencer_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "utilities/can_utils.hpp"
#include "utilities/utils.hpp"
#include <fstream>
#include <boost/format.hpp>

using namespace std;
using namespace utils;

namespace sequencer{

Sequencer::Sequencer(const rclcpp::NodeOptions &options) : Sequencer("", options) {}

Sequencer::Sequencer(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("sequencer_node", name_space, options),
can_movable_id(get_parameter("canid.movable").as_int()),
can_digital_button_id(get_parameter("canid.sub_digital_button").as_int()),
can_inject_id(get_parameter("canid.inject").as_int()),

socket_robot_state(get_parameter("port.robot_state").as_int()),
socket_pole_state(get_parameter("port.pole_state").as_int()),

pole_priorityA_file_path(ament_index_cpp::get_package_share_directory("main_executor")+"/config/"+"/sequencer/"+"pole_priority_A.cfg"),
pole_priorityB_file_path(ament_index_cpp::get_package_share_directory("main_executor")+"/config/"+"/sequencer/"+"pole_priority_B.cfg"),
pole_priorityC_file_path(ament_index_cpp::get_package_share_directory("main_executor")+"/config/"+"/sequencer/"+"pole_priority_C.cfg")

{
    _subscription_base_control = this->create_subscription<controller_interface_msg::msg::BaseControl>(
        "pub_base_control",
        _qos,
        std::bind(&Sequencer::_subscriber_callback_base_control, this, std::placeholders::_1)
    );
    _subscription_convergence = this->create_subscription<controller_interface_msg::msg::Convergence>(
        "pub_convergence",
        _qos,
        std::bind(&Sequencer::_subscriber_callback_convergence, this, std::placeholders::_1)
    );
    _subscription_movable = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_"+ (boost::format("%x") % can_movable_id).str(),
        _qos,
        std::bind(&Sequencer::_subscriber_callback_movable, this, std::placeholders::_1)
    );

    _socket_timer = this->create_wall_timer(
        std::chrono::milliseconds(this->get_parameter("interval_ms").as_int()),
        [this] { _recv_callback(); }
    );

    publisher_can = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    publisher_pole_m0 = this->create_publisher<std_msgs::msg::String>("injection_pole_m0", _qos);
    publisher_pole_m1 = this->create_publisher<std_msgs::msg::String>("injection_pole_m1", _qos);
    publisher_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);

    pole_priority_m0.reserve(11);
    pole_priority_m1.reserve(11);
}

void Sequencer::_subscriber_callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart){
        current_inject_state = msg->initial_state;
        current_pickup_state = msg->initial_state;
    }
    judge_convergence.spline_convergence = msg->is_move_autonomous;

    // judge_convergence.injection0 = msg->is_injection_autonomous;
    // judge_convergence.injection1 = msg->is_injection_autonomous;
}

void Sequencer::_subscriber_callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){
    if(current_pickup_state == "L0" || current_pickup_state == "L1"){
        if(msg->spline_convergence && judge_convergence.spline_convergence){
            auto msg_load = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_load->canid = can_digital_button_id;
            msg_load->candlc = 8;
            msg_load->candata[5] = true;    //装填
            publisher_can->publish(*msg_load);
        }
        // if(msg->injection0 && msg->injection1){
        //     _recv_pole_state(last_pole_state);
        //     current_pickup_state = "O";
        // }
    }
    // if(current_pickup_state == "O"){
    else{
        auto msg_inject = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_inject->canid = can_inject_id;
        msg_inject->candlc = 2;

        if(is_auto_inject_m0 && msg->injection_calculator0 && msg->injection0 && (msg->spline_convergence || !judge_convergence.spline_convergence)){
            msg_inject->candata[0] = true;
            publisher_can->publish(*msg_inject);
        }
        if(is_auto_inject_m1 && msg->injection_calculator1 && msg->injection1 && (msg->spline_convergence || !judge_convergence.spline_convergence)){
            msg_inject->candata[1] = true;
            publisher_can->publish(*msg_inject);
        }
    }
}

void Sequencer::_subscriber_callback_movable(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    if(current_pickup_state == "L0" || current_pickup_state == "L1"){
        auto msg_move_node = std::make_shared<std_msgs::msg::String>();
        msg_move_node->data = current_inject_state;
        publisher_move_node->publish(*msg_move_node);

        current_pickup_state = "O";
        _recv_pole_state(last_pole_state);
    }
}

void Sequencer::_recv_callback(){
    if(socket_robot_state.is_recved()){
        unsigned char data[2];
        _recv_robot_state(socket_robot_state.data(data, sizeof(data)));
    }
    if(socket_pole_state.is_recved()){
        unsigned char data[11] = {0};
        _recv_pole_state(socket_pole_state.data(data, sizeof(data)));
    }
}

void Sequencer::_recv_robot_state(const unsigned char data[2]){
    const string state = {static_cast<char>(data[0]), static_cast<char>(data[1])};

    auto msg_pickup_preparation = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_pickup_preparation->canid = can_digital_button_id;
    msg_pickup_preparation->candlc = 8;

    auto msg_move_node = std::make_shared<std_msgs::msg::String>();

    if(state=="L0"){
        current_pickup_state = state;
        msg_move_node->data = state;
        publisher_move_node->publish(*msg_move_node);

        msg_pickup_preparation->candata[6] = true; //左回収準備
        publisher_can->publish(*msg_pickup_preparation);
        return;
    }
    else if(state=="L1"){
        current_pickup_state = state;
        msg_move_node->data = state;
        publisher_move_node->publish(*msg_move_node);

        msg_pickup_preparation->candata[4] = true; //右回収準備
        publisher_can->publish(*msg_pickup_preparation);
        return;
    }

    /*回収特殊状態はこの先は実行しない*/

    string pole_priority_file_path;
    current_inject_state = state[0];

    if(state[0]=='A') pole_priority_file_path = pole_priorityA_file_path;
    else if(state[0]=='B') pole_priority_file_path = pole_priorityB_file_path;
    else if(state[0]=='C') pole_priority_file_path = pole_priorityC_file_path;

    ifstream ifs(pole_priority_file_path);
    string str;
    int mech_num = 0;
    while(getline(ifs, str)){
        string token;
        istringstream stream(str);
        int count = 0;
        while(getline(stream, token, ' ')){   //スペース区切り
            if(count==0 && token=="#") break;
            else if(count==0) mech_num++;

            if(mech_num==1) pole_priority_m0.push_back(static_cast<int>(token[0])-0x41);
            else if(mech_num==2) pole_priority_m1.push_back(static_cast<int>(token[0])-0x41);
            count++;
        }
    }
    if(current_pickup_state != "L0" && current_pickup_state != "L1"){
        auto msg_move_node = std::make_shared<std_msgs::msg::String>();
        msg_move_node->data = current_inject_state;
        publisher_move_node->publish(*msg_move_node);
    }

}

void Sequencer::_recv_pole_state(const unsigned char data[11]){
    auto injection_pole_m0 = std::make_shared<std_msgs::msg::String>();
    auto injection_pole_m1 = std::make_shared<std_msgs::msg::String>();

    // RCLCPP_INFO(this->get_logger(), "pole %d %d %d %d %d %d %d %d %d %d %d ", data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10]);
    if(current_pickup_state != "L0" && current_pickup_state != "L1"){

    is_auto_inject_m0 = false;
    is_auto_inject_m1 = false;

    for(const auto& pole_num : pole_priority_m0){
        if(!data[pole_num]){
            if(pole_num != aiming_pole_num_m0){
                string pole{static_cast<char>(pole_num+0x41)};  //0~10をA~Kに変換
                injection_pole_m0->data = pole;
                publisher_pole_m0->publish(*injection_pole_m0);
                aiming_pole_num_m0 = pole_num;
            }
            is_auto_inject_m0 = true;
            break;
        }
    }
    for(const auto& pole_num : pole_priority_m1){
        if(!data[pole_num]){
            if(pole_num != aiming_pole_num_m1){
                string pole{static_cast<char>(pole_num+0x41)};  //0~10をA~Kに変換
                injection_pole_m1->data = pole;
                publisher_pole_m1->publish(*injection_pole_m1);
                aiming_pole_num_m1 = pole_num;
            }
            is_auto_inject_m1 = true;
            break;
        }
    }

    }
    memcpy(last_pole_state, data, sizeof(last_pole_state));
}

}  // namespace sequencer
