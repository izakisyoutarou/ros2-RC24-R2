#include "sequencer/sequencer_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>

using namespace utils;

namespace sequencer{

Sequencer::Sequencer(const rclcpp::NodeOptions &options) : Sequencer("", options) {}

Sequencer::Sequencer(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("sequencer_node", name_space, options),

        court_color(get_parameter("court_color").as_string()),
        can_paddy_collect_id(get_parameter("canid.paddy_collect").as_int()),
        can_paddy_install_id(get_parameter("canid.paddy_install").as_int()),
        can_net_id(get_parameter("canid.net").as_int()),
        can_hand_lift_id(get_parameter("canid.hand_lift").as_int()),
        can_hand_fb_id(get_parameter("canid.hand_fb").as_int()),
        can_hand_wrist_id(get_parameter("canid.hand_wrist").as_int()),
        can_hand_suction_id(get_parameter("canid.suction").as_int()),
        can_tof_id(get_parameter("canid.tof").as_int())

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

    _subscription_way_point = this->create_subscription<std_msgs::msg::String>(
        "way_point",
        _qos,
        std::bind(&Sequencer::callback_way_point, this, std::placeholders::_1)
    );

    _subscription_self_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
        "self_pose",
        rclcpp::SensorDataQoS(),
        std::bind(&Sequencer::callback_self_pose, this, std::placeholders::_1)
    );

    _subscription_front_ball = this->create_subscription<std_msgs::msg::Bool>(
        "front_ball",
        _qos,
        std::bind(&Sequencer::callback_front_ball, this, std::placeholders::_1)
    );

    _subscription_tof = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
        "can_rx_"+ (boost::format("%x") % can_tof_id).str(),
        _qos,
        std::bind(&Sequencer::callback_tof, this, std::placeholders::_1)
    );

    _subscription_ball_coordinate = this->create_subscription<geometry_msgs::msg::Vector3>(
        "ball_coordinate",
        _qos,
        std::bind(&Sequencer::callback_ball_coordinate, this, std::placeholders::_1)
    );

    _subscription_siro_param = this->create_subscription<detection_interface_msg::msg::SiroParam>(
        "siro_param",
        _qos,
        std::bind(&Sequencer::callback_siro_param, this, std::placeholders::_1)
    );

    _publisher_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
    _publisher_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    _publisher_now_sequence = this->create_publisher<std_msgs::msg::String>("now_sequence", _qos);
    _publisher_move_interrupt_node = this->create_publisher<std_msgs::msg::String>("move_interrupt_node", _qos);
    _pub_spin_position = this->create_publisher<path_msg::msg::Turning>("spin_position", 10);    
    _publisher_ball_tracking = this->create_publisher<geometry_msgs::msg::Vector3>("ball_tracking", _qos);

    std::ifstream ifs1(ament_index_cpp::get_package_share_directory("main_executor") + "/config/sequencer/silo_priority.cfg");
    std::string str1;
    int n = 0;
    while(getline(ifs1, str1)){
        std::string token;
        std::istringstream stream(str1);
        int count = 0;
        while(getline(stream, token, ' ')){
            if(token == "V") silo_norm[n][count] = "";
            else silo_norm[n][count] = token;
            count++;
        }
        n++;
    }

    command_sequence(SEQUENCE_MODE::stop);

}

void Sequencer::callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){
    int n = 0;
    if(!is_start) return;
    if(sequence_mode == SEQUENCE_MODE::storage){
        if(progress == n++){
            command_move_node("c2");
            command_hand_fb_front();
            command_hand_lift_pickup();
            command_hand_wrist_down();
            command_hand_suction_on();
            suction_time = std::chrono::system_clock::now();
            progress++;
        } 
        else if(progress == n++ && msg->spline_convergence && msg->arm_convergence){
            if(front_ball) command_hand_fb_front();
            else command_hand_fb_back();
            command_hand_lift_suction_before();
            progress++;
        }
        else if(progress == n++ && msg->arm_convergence && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - suction_time).count() > 5000){
            command_hand_lift_suction();
            suction_time = std::chrono::system_clock::now();
            progress++;       
        }
        else if(progress == n++ && msg->arm_convergence && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - suction_time).count() > 2000){
            command_hand_lift_pickup();
            progress++;
        }
        else if(progress == n++ && msg->arm_convergence){
            if(tof_mode){
                if(tof[0]){
                    command_sequence(SEQUENCE_MODE::silo);
                    ball_num++;                       
                }
                else progress--;            
                progress++;
            }
            else command_sequence(SEQUENCE_MODE::silo);
        }
    }
    else if(sequence_mode == SEQUENCE_MODE::transfer){
        if(progress == n++){
            command_move_interrupt_node("ST8");
            command_hand_fb_inside();
            command_hand_lift_pickup();
            command_hand_wrist_down();
            command_net_open();
            progress++;
        }
        else if(progress == n++ && msg->spline_convergence && msg->net_convergence){
            command_net_close();
            command_hand_suction_on();
            suction_time = std::chrono::system_clock::now();
            progress++;
        }
        else if(progress == n++ && msg->net_convergence && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - suction_time).count() > 5000){
            command_hand_lift_inside();
            suction_time = std::chrono::system_clock::now();
            progress++;
        }
        else if(progress == n++ && msg->arm_convergence && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - suction_time).count() > 2000){
            if(tof_mode){
                if(tof[0]){
                    command_sequence(SEQUENCE_MODE::silo);
                    ball_num++;                       
                }
                else progress--;            
                progress++;
            }
            else command_sequence(SEQUENCE_MODE::silo);
            progress++;
        }
    }
    else if(sequence_mode == SEQUENCE_MODE::collect){
        if(progress == n++){
            command_move_node("c2");
            command_hand_lift_suction_before();
            command_hand_fb_front();
            command_hand_wrist_down();
            command_hand_suction_on();
            suction_time = std::chrono::system_clock::now();
            progress++;
        }
        else if(progress == n++ && get_ball_pose){
            auto ball_tracking = std::make_shared<geometry_msgs::msg::Vector3>();
            float length = 0.3 + 0.095;
            if(ball_pose.x > self_pose.x){
                ball_tracking->x = ball_pose.x - length*sin(ball_pose.z);
                ball_tracking->y = ball_pose.y - length*cos(ball_pose.z);
                ball_tracking->z = ball_pose.z;
            }
            else if(self_pose.x > ball_pose.x){
                ball_tracking->x = ball_pose.x + length*sin(ball_pose.z);
                ball_tracking->y = ball_pose.y - length*cos(ball_pose.z);
                ball_tracking->z = ball_pose.z;                        
            }
            _publisher_ball_tracking->publish(*ball_tracking);
            get_ball_pose = false;
            progress++;
        } 
        else if(progress == n++ && msg->spline_convergence && msg->arm_convergence && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - suction_time).count() > 5000){
            command_hand_lift_suction();
            suction_time = std::chrono::system_clock::now();
            progress++;
        } 
        else if(progress == n++ && msg->arm_convergence && std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - suction_time).count() > 2000){
            command_hand_lift_pickup();
            progress++;
        }
        else if(progress == n++ && msg->arm_convergence) {
            if(tof_mode){
                if(tof[0]){
                    command_sequence(SEQUENCE_MODE::silo);
                    ball_num++;                       
                }
                else progress--;
            }
            else command_sequence(SEQUENCE_MODE::silo);
        }
    }
    else if(sequence_mode == SEQUENCE_MODE::silo){
        if(tof_mode){        
            if(progress == 1 && !tof[0]){
                if(pre_sequence == SEQUENCE_MODE::storage) {
                    if(ball_num < 6) {
                        command_sequence(SEQUENCE_MODE::storage);
                        command_move_interrupt_node("c1");
                        progress = 1;
                    }
                    else {
                        command_sequence(SEQUENCE_MODE::storage);
                        command_move_interrupt_node("ST8");
                        progress = 1;
                        ball_num = 0;                        
                    }
                }
                else if(pre_sequence == SEQUENCE_MODE::transfer) {
                    if(ball_num < 6) {
                        command_sequence(SEQUENCE_MODE::storage);
                        command_move_interrupt_node("ST8");
                        progress = 1;
                    }
                    else {
                        command_sequence(SEQUENCE_MODE::collect);
                        ball_num = 0;                    
                    }
                }
                else command_sequence(SEQUENCE_MODE::collect);
            }
        }
        if(progress == n++){
            command_move_node("c1");
            command_hand_lift_pickup();
            progress++;
        }
        else if(progress == n++ && msg->arm_convergence){
            command_hand_fb_silo();
            progress++;
        }
        
        else if(progress == n++&& msg->arm_convergence){
            command_hand_lift_silo();
            progress++;
        }
        else if(progress == n++&& msg->arm_convergence){
            command_hand_wrist_up();
            progress++;
        }
        else if(progress == n++ && silo_flag){
            command_move_node("SI" + std::to_string(target_silo));
            silo_flag = false;
            progress++;
        } 
        else if(progress == n++ && msg->spline_convergence){
            command_hand_suction_off();
            suction_time = std::chrono::system_clock::now();            
            progress++;
        } 
        else if(progress == n++ && msg->arm_convergence && !tof[0]) {
            if(pre_sequence == SEQUENCE_MODE::storage) {
                if(ball_num < 6) command_sequence(SEQUENCE_MODE::storage);
                else {
                    command_sequence(SEQUENCE_MODE::transfer);
                    ball_num = 0;                        
                }
            }
            else if(pre_sequence == SEQUENCE_MODE::transfer) {
                if(ball_num < 6) command_sequence(SEQUENCE_MODE::transfer);
                else{
                    command_sequence(SEQUENCE_MODE::collect);
                    ball_num = 0;
                }
            }
            else command_sequence(SEQUENCE_MODE::collect);
        }
    }        
}

void Sequencer::callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart) command_sequence(SEQUENCE_MODE::stop);
    is_start = false;
    progress = 0;
}

void Sequencer::callback_is_start(const std_msgs::msg::UInt8::SharedPtr msg){
    if(msg->data == 0) command_sequence(SEQUENCE_MODE::storage);
    else if(msg->data == 1) command_sequence(SEQUENCE_MODE::transfer);
    else if(msg->data == 2) command_sequence(SEQUENCE_MODE::collect);
    else if(msg->data == 3) command_sequence(SEQUENCE_MODE::silo);
    is_start = true;
}

void Sequencer::callback_collection_point(const std_msgs::msg::String::SharedPtr msg){
    if(msg->data == "" && sequence_mode == SEQUENCE_MODE::storage) command_sequence(SEQUENCE_MODE::transfer);
    else command_move_interrupt_node(msg->data);
}

void Sequencer::callback_way_point(const std_msgs::msg::String::SharedPtr msg){
    way_point = msg->data;
}

void Sequencer::callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
    this->self_pose.x = msg->x;
    this->self_pose.y = msg->y;
    this->self_pose.z = msg->z;
}

void Sequencer::callback_ball_coordinate(const geometry_msgs::msg::Vector3::SharedPtr msg){
    this->ball_pose.x = msg->x;
    this->ball_pose.y = msg->y;
    this->ball_pose.z = msg->z;
    get_ball_pose = true;
}

void Sequencer::callback_front_ball(const std_msgs::msg::Bool::SharedPtr msg){
    front_ball = msg->data;
    get_front_ball = true;
}

void Sequencer::callback_tof(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    tof[0] = msg->candata[0];
    tof[1] = msg->candata[1];
    tof[2] = msg->candata[2];
};

void Sequencer::callback_siro_param(const detection_interface_msg::msg::SiroParam::SharedPtr msg){
    std::string ball_color[15];
    for(int i = 0; i < 15; i++) ball_color[i] = msg->ball_color[i];
    target_silo = silo_evaluate(ball_color);
    silo_flag = true;
}

void Sequencer::command_move_node(const std::string node){
    auto msg_move_node = std::make_shared<std_msgs::msg::String>();
    msg_move_node->data = node;
    _publisher_move_node->publish(*msg_move_node);
}

void Sequencer::command_move_interrupt_node(const std::string node){
    auto msg_move_interrupt_node = std::make_shared<std_msgs::msg::String>();
    msg_move_interrupt_node->data = node;
    _publisher_move_interrupt_node->publish(*msg_move_interrupt_node);
}

void Sequencer::command_spin_position(const float angle){
    auto spin_angle = std::make_shared<path_msg::msg::Turning>();
    spin_angle->angle_pos = angle;
    spin_angle->accurate_convergence = true;
    _pub_spin_position->publish(*spin_angle);
};

void Sequencer::command_sequence(const SEQUENCE_MODE sequence){
    pre_sequence = sequence_mode;
    sequence_mode = sequence;
    auto msg_now_sequence = std::make_shared<std_msgs::msg::String>();
    msg_now_sequence->data = sequence_list[static_cast<int>(sequence)];
    _publisher_now_sequence->publish(*msg_now_sequence);
    progress = 0;
}

void Sequencer::command_canusb_uint8(const int16_t id, const uint8_t data){
    auto msg_canusb = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_canusb->canid = id;
    msg_canusb->candlc = 1;
    msg_canusb->candata[0] = data;
    _publisher_canusb->publish(*msg_canusb);
}

void Sequencer::command_canusb_empty(const int16_t id){
    auto msg_canusb = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_canusb->canid = id;
    msg_canusb->candlc = 0;
    _publisher_canusb->publish(*msg_canusb);
}

void Sequencer::command_paddy_collect_front(){ command_canusb_uint8(can_paddy_collect_id, 0); }
void Sequencer::command_paddy_collect_back(){ command_canusb_uint8(can_paddy_collect_id, 1); }
void Sequencer::command_paddy_install(){ command_canusb_empty(can_paddy_install_id); }
void Sequencer::command_net_open(){ command_canusb_uint8(can_net_id, 0); }
void Sequencer::command_net_close(){ command_canusb_uint8(can_net_id, 1); }
void Sequencer::command_hand_lift_suction_before(){ command_canusb_uint8(can_hand_lift_id, 0); }
void Sequencer::command_hand_lift_suction(){ command_canusb_uint8(can_hand_lift_id, 1); }
void Sequencer::command_hand_lift_pickup(){ command_canusb_uint8(can_hand_lift_id, 2); }
void Sequencer::command_hand_lift_inside(){ command_canusb_uint8(can_hand_lift_id, 3); }
void Sequencer::command_hand_lift_silo(){ command_canusb_uint8(can_hand_lift_id, 4); }
void Sequencer::command_hand_fb_front(){ command_canusb_uint8(can_hand_fb_id, 0); }
void Sequencer::command_hand_fb_back(){ command_canusb_uint8(can_hand_fb_id, 1); }
void Sequencer::command_hand_fb_inside(){ command_canusb_uint8(can_hand_fb_id, 2); }
void Sequencer::command_hand_fb_silo(){ command_canusb_uint8(can_hand_fb_id, 3); }
void Sequencer::command_hand_wrist_up(){ command_canusb_uint8(can_hand_wrist_id, 0); }
void Sequencer::command_hand_wrist_down(){ command_canusb_uint8(can_hand_wrist_id, 1); }
void Sequencer::command_hand_suction_on(){ command_canusb_uint8(can_hand_suction_id, 0); }
void Sequencer::command_hand_suction_off(){ command_canusb_uint8(can_hand_suction_id, 1); }

int Sequencer::silo_evaluate(std::string camera[15]){
  
    //コート色による情報反転
    if(court_color == "blue"){
        std::string data[15]; 
        int num[15] = {12,13,14,9,10,11,6,7,8,3,4,5,0,1,2};
        for(int i = 0; i < 15; i++) data[i] = camera[i];
        for(int i = 0; i < 15; i++) camera[i] = data[num[i]];
    }    
    
    //データの格納
    for(int i = 0; i < 15; i++) {
        if(court_color == "blue"){
            if(camera[i] == "B") silo_data[i/3][i%3] = "M";
            else if(camera[i] == "R") silo_data[i/3][i%3] = "E";
            else silo_data[i/3][i%3] = "";
        }
        else if(court_color == "red"){
            if(camera[i] == "B") silo_data[i/3][i%3] = "E";
            else if(camera[i] == "R") silo_data[i/3][i%3] = "M";
            else silo_data[i/3][i%3] = "";
        }
    }

    //評価判定
    for(int i = 0; i < 5; i++){
        bool check = false;
        for(int j = 0; j < 11; j++){
            if(silo_data[i][0] == silo_norm[j][0] && silo_data[i][1] == silo_norm[j][1] && silo_data[i][2] == silo_norm[j][2]) {
                silo_priority[i] = std::stoi(silo_norm[j][3]);
                check = true;
                break;
            }
        }
        if(!check) silo_priority[i] = 10;
    }

    //評価比較
    int silo_num = 0;
    int priority_min = silo_priority[0];
    for(int i = 1; i < 5; i++){
        if(silo_priority[i] < priority_min){
            priority_min = silo_priority[i];
            silo_num = i;
        }
    }
         
    return silo_num;
}

}  // namespace sequencer