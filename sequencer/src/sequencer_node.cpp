#include "sequencer/sequencer_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <fstream>
#include <cmath>
#include <boost/format.hpp>

#include "utilities/can_utils.hpp"

using namespace utils;

namespace sequencer{

Sequencer::Sequencer(const rclcpp::NodeOptions &options) : Sequencer("", options) {}

Sequencer::Sequencer(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("sequencer_node", name_space, options),

        can_net_id(get_parameter("canid.net").as_int()),
        can_tof_id(get_parameter("canid.tof").as_int()),
        can_hand_suction_id(get_parameter("canid.suction").as_int()),
        can_base_state_id(get_parameter("canid.base_state").as_int()),
        can_strage_state_id(get_parameter("canid.strage_state").as_int()),
        can_strage_state2_id(get_parameter("canid.strage_state2").as_int()),
        can_transfer_state_id(get_parameter("canid.transfer_state").as_int()),
        can_silo_state_id(get_parameter("canid.silo_state").as_int()),
        can_silo_state2_id(get_parameter("canid.silo_state2").as_int()),
        can_hand_lift_id(get_parameter("canid.hand_lift").as_int()),
        court_color(get_parameter("court_color").as_string()),
        strage_dist(get_parameter("strage_dist").as_double_array()),
        suction_wait(get_parameter("suction_wait").as_double()),
        silo_wait(get_parameter("silo_wait").as_double())

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

    _subscription_silo_param = this->create_subscription<detection_interface_msg::msg::SiloParam>(
        "silo_param",
        _qos,
        std::bind(&Sequencer::callback_silo_param, this, std::placeholders::_1)
    );

    _subscription_suction_check = this->create_subscription<std_msgs::msg::String>(
        "suction_check",
        _qos,
        std::bind(&Sequencer::callback_suction_check, this, std::placeholders::_1)
    );

    _subscription_move_progress = this->create_subscription<std_msgs::msg::Float64>(
        "move_progress",
        _qos,
        std::bind(&Sequencer::callback_move_progress, this, std::placeholders::_1)
    );

    _publisher_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
    _publisher_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    _publisher_now_sequence = this->create_publisher<std_msgs::msg::String>("now_sequence", _qos);
    _publisher_move_interrupt_node = this->create_publisher<std_msgs::msg::String>("move_interrupt_node", _qos);
    _publisher_coord_tracking = this->create_publisher<geometry_msgs::msg::Vector3>("coord_tracking", _qos);
    _publisher_move_autonomous = this->create_publisher<std_msgs::msg::Bool>("move_autonomous", _qos);

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
    
    if(!is_start && !move_automonous) return;
    int n = 0;

    if(sequence_mode == SEQUENCE_MODE::storage){
        if(progress == n++){
            if(c3b_flag) {
                RCLCPP_INFO(get_logger(),"_____strage0_c3b____");
                command_move_node("c3_b");
            }
            else if(c3a_flag){
                RCLCPP_INFO(get_logger(),"_____strage0_c3a____");
                command_move_node("c3_a");
            }
            else if(c6a_flag){
                RCLCPP_INFO(get_logger(),"_____strage0_c6a____");
                command_move_node("c6_a");
            }
            else {
                RCLCPP_INFO(get_logger(),"_____strage0_c6b____");
                command_move_node("c6_b");
            }
            get_ball_pose = false;
            timer(1000);
            progress++;
        }
        else if(progress == n++ && timer()){
            RCLCPP_INFO(get_logger(),"_____strage1_____");
            command_base_state();
            progress++;
        }
        else if(progress == n++ && msg->arm_convergence && !msg->spline_convergence && get_ball_pose){
            RCLCPP_INFO(get_logger(),"_____strage2_____");
            command_hand_suction_on();
            command_ball_tracking();
            progress++;
        }
        else if(progress == n++ && way_point == "coord"){
            RCLCPP_INFO(get_logger(),"_____strage3_____");
            command_strage_state(kado_flag);
            progress++;
        }
        else if(progress == n++ && get_suction_check && msg->arm_convergence){
            RCLCPP_INFO(get_logger(),"suction:%s",suction_check.c_str());
            if(suction_check == "M"){
                RCLCPP_INFO(get_logger(),"_____strage4_M_____");
                command_strage_state2();
                command_sequence(SEQUENCE_MODE::silo); 
            }
            else {
                RCLCPP_INFO(get_logger(),"_____strage4_P[]_____");
                progress = n++;
            }
        }
        else if(progress == n++){
            RCLCPP_INFO(get_logger(),"_____strage5_____");
            command_hand_suction_off();
            timer(400);
            progress++;
        }
        else if(progress == n++ && timer()){
            command_base_state();
            get_suction_check = false;
            get_ball_pose = false;
            if(c3b_flag) {
                RCLCPP_INFO(get_logger(),"_____strage6_c3b____");
                command_move_interrupt_node("c3_b");
            }
            else if(c3a_flag){
                RCLCPP_INFO(get_logger(),"_____strage6_c3a____");
                command_move_interrupt_node("c3_a");
            }
            else if(c6a_flag){
                RCLCPP_INFO(get_logger(),"_____strage6_c6a____");
                command_move_interrupt_node("c6_a");
            }
            else {
                RCLCPP_INFO(get_logger(),"_____strage6_c6b____");
                command_move_interrupt_node("c6_b");
            }
            progress -= 4;
        }
    }

    else if(sequence_mode == SEQUENCE_MODE::transfer){
        if(progress == n++){
            RCLCPP_INFO(get_logger(),"_____transfer0_____");
            command_move_node("ST8");
            command_net_open();
            command_hand_lift_pickup();
            progress++;
        }
        else if(progress == n++ && msg->net_convergence && way_point == "ST8"){
            RCLCPP_INFO(get_logger(),"_____transfer1_____");
            command_net_close();
            progress++;
        }
        else if(progress == n++){
            // if(tof[0] || tof[1]){
            //     RCLCPP_INFO(get_logger(),"_____transfer2_____");
            //     command_sequence(SEQUENCE_MODE::silo);
            // }

            // else if(msg->net_convergence){
            //     net_flag = !net_flag;
            //     if(net_flag) command_net_open();
            //     else command_net_close();
            // }
            if(tof[4]){
                RCLCPP_INFO(get_logger(),"_____transfer2_____");
                command_sequence(SEQUENCE_MODE::silo);
            }
            else if(msg->net_convergence){
                net_flag = !net_flag;
                if(net_flag) command_net_open();
                else command_net_close();
            }
        }
    }

    else if(sequence_mode == SEQUENCE_MODE::collect){
        // if(progress == n++){
        //     RCLCPP_INFO(get_logger(),"_____collect0_____");
        //     command_move_node("c2");
        //     progress++;
        // }
        // else if(progress == n++ && (way_point == "c7" || way_piint == "c8")){
        //     RCLCPP_INFO(get_logger(),"_____collect1_____");
        //     get_ball_pose = false;
        //     progress++;
        // }
        // else if(progress == n++ && msg->arm_convergence && get_ball_pose){
        //     RCLCPP_INFO(get_logger(),"_____strage2_____");
        //     command_hand_suction_on();
        //     command_ball_tracking();
        //     progress++;
        // }
        // else if(progress == n++ && way_point == "coord"){
        //     RCLCPP_INFO(get_logger(),"_____strage4_____");
        //     command_strage_state(false);
        //     progress++;
        // }
        // else if(progress == n++ && get_suction_check && msg->arm_convergence){
        //     suction_check = "M";
        //     if(suction_check == "M"){
        //         RCLCPP_INFO(get_logger(),"_____strage5_M_____");
        //         command_strage_state2();
        //         command_sequence(SEQUENCE_MODE::silo); 
        //         get_suction_check = false;
        //     }
        //     else {
        //         RCLCPP_INFO(get_logger(),"_____strage5_P[]_____");
        //         command_hand_suction_off();
        //         command_strage_state2();
        //         get_ball_pose = false
        //         progress -= 2;
        //     }
        // }
    }

    else if(sequence_mode == SEQUENCE_MODE::silo){
        // if(progress == n++){
        //     RCLCPP_INFO(get_logger(),"_____silo0_____");
        //     if(pre_sequence == SEQUENCE_MODE::storage){
        //         if(c4_flag) command_move_interrupt_node("c6");
        //         else  command_move_interrupt_node("c3");
        //         c3orc6_flag = false;
        //     }
        //     else if(pre_sequence == SEQUENCE_MODE::transfer) c3orc6_flag = true;
        //     progress++;
        // }
        if(progress == n++){
            RCLCPP_INFO(get_logger(),"_____silo1_____");
            command_move_node("c1");
            progress++;
        }
        else if(progress == n++){
            if(pre_sequence == SEQUENCE_MODE::transfer && tof[1]){
                RCLCPP_INFO(get_logger(),"_____silo2_transfer_____");
                command_hand_suction_on();
                command_transfer_state();
                progress++;
            } 
            else if(pre_sequence != SEQUENCE_MODE::transfer) {
                RCLCPP_INFO(get_logger(),"_____silo2_not_transfer_____");
                progress++;
            }
        }
        else if(progress == n++){
            RCLCPP_INFO(get_logger(),"_____silo3_____");
            if(pre_sequence == SEQUENCE_MODE::transfer && msg->arm_convergence){
                RCLCPP_INFO(get_logger(),"_____silo3_transfer_____");
                command_hand_lift_pickup();
                progress++;
            } 
            else if(pre_sequence != SEQUENCE_MODE::transfer) {
                RCLCPP_INFO(get_logger(),"_____silo3_not_transfer_____");
                progress++;
            }
        }
        else if(progress == n++ && c1_flag && msg->arm_convergence && silo_flag){
            RCLCPP_INFO(get_logger(),"_____silo4_____");
            command_silo_state();
            c1_flag = false;
            silo_flag = false;
            priority_num = 0;
            progress++;
        }       
        else if(progress == n++){
            RCLCPP_INFO(get_logger(),"_____silo5_____");
            std::string silo_node = "si" + std::to_string(silo_priority[priority_num]);
            command_move_interrupt_node(silo_node);
            progress++; 
        } 
        else if(progress == n++ && way_point ==  interrupt_node){
            if(priority_num == 4){
                RCLCPP_INFO(get_logger(),"_____silo6_last_____");
                std::string silo_node = "SI" + std::to_string(silo_priority[priority_num]);
                command_move_interrupt_node(silo_node);
                progress++; 
            } 
            else if(tof[2]){
                RCLCPP_INFO(get_logger(),"_____silo6_change______");
                priority_num++;
                progress--; 
            } 
            else {
                RCLCPP_INFO(get_logger(),"_____silo6_go_____");
                std::string silo_node = "SI" + std::to_string(silo_priority[priority_num]);
                command_move_interrupt_node(silo_node);
                progress++;
            }
        }
        else if(progress == n++ && check_way_SI()){
            RCLCPP_INFO(get_logger(),"_____silo7_____");
            command_hand_suction_off();
            timer(400);
            progress++;
        } 
        else if(progress == n++ && msg->arm_convergence && timer()) {
            command_silo_state2();
            // if(pre_sequence == SEQUENCE_MODE::storage) {
            //     ball_num++;
            //     if(ball_num == 3) command_sequence(SEQUENCE_MODE::transfer);
            //     else command_sequence(SEQUENCE_MODE::storage);
            // }
            if(pre_sequence == SEQUENCE_MODE::storage)  command_sequence(SEQUENCE_MODE::storage);
            else if(pre_sequence == SEQUENCE_MODE::transfer) command_sequence(SEQUENCE_MODE::transfer);
        }
    }
} 

void Sequencer::callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart) {
        command_sequence(SEQUENCE_MODE::stop);
        is_start = false;
        c1_flag = false;
        way_point = "";
        progress = 0;
        c3a_flag = false;
        c3b_flag = false;
        c6a_flag = false;
        c6b_flag = false;
        ball_num = 0;
    }
    if(msg->initial_state == "O") retry_flag = false;
    else if(msg->initial_state == "A") retry_flag = true;
    move_automonous = msg->is_move_autonomous;
}

void Sequencer::callback_is_start(const std_msgs::msg::UInt8::SharedPtr msg){
    if(msg->data == 0) command_sequence(SEQUENCE_MODE::storage);
    else if(msg->data == 1) command_sequence(SEQUENCE_MODE::transfer);
    else if(msg->data == 2) command_sequence(SEQUENCE_MODE::collect);
    else if(msg->data == 3) command_sequence(SEQUENCE_MODE::silo);
    is_start = true;
    command_move_autonomous(true);
}

void Sequencer::callback_collection_point(const std_msgs::msg::String::SharedPtr msg){
    if((!retry_flag && way_point == "c3_b" && msg->data == "" && sequence_mode == SEQUENCE_MODE::storage) || (retry_flag && way_point == "c3_b" && msg->data == "" && sequence_mode == SEQUENCE_MODE::storage)) command_sequence(SEQUENCE_MODE::transfer);
    else if(msg->data == "c3_a" && sequence_mode == SEQUENCE_MODE::storage) command_move_node("c3_a");
    else command_move_interrupt_node(msg->data);
    if(msg->data == "c3_a") c3a_flag = true;
    if(msg->data == "c3_b") c3b_flag = true;
    if(msg->data == "c6_a") c6a_flag = true;
    if(msg->data == "c6_b") c6b_flag = true;
}

void Sequencer::callback_way_point(const std_msgs::msg::String::SharedPtr msg){
    way_point = msg->data;
    if(msg->data == "c1" && sequence_mode == SEQUENCE_MODE::silo) c1_flag = true;
}

void Sequencer::callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
    this->self_pose.x = msg->x;
    this->self_pose.y = msg->y;
    this->self_pose.z = msg->z;
}

void Sequencer::callback_ball_coordinate(const geometry_msgs::msg::Vector3::SharedPtr msg){
    this->ball_pose.x = msg->x;
    this->ball_pose.y = msg->y;
    get_ball_pose = true;
}

void Sequencer::callback_tof(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    tof[0] = msg->candata[0];
    tof[1] = msg->candata[1];
    tof[2] = msg->candata[2];
    tof[3] = msg->candata[3];
};

void Sequencer::callback_silo_param(const detection_interface_msg::msg::SiloParam::SharedPtr msg){
    if(sequence_mode == SEQUENCE_MODE::silo  && !silo_flag){
        std::string ball_color[15];
        for(int i = 0; i < 15; i++) ball_color[i] = msg->ball_color[i];
        silo_evaluate(ball_color);
    }
}

void Sequencer::callback_suction_check(const std_msgs::msg::String::SharedPtr msg){
    if(msg->data == "B" || msg->data == "R") suction_check = "M";
    else suction_check = msg->data; 
    get_suction_check = true;
}

void Sequencer::callback_move_progress(const std_msgs::msg::Float64::SharedPtr msg){
    if(msg->data >= 0.5) move_progress = true;
}

void Sequencer::command_move_node(const std::string node){
    auto msg_move_node = std::make_shared<std_msgs::msg::String>();
    msg_move_node->data = node;
    _publisher_move_node->publish(*msg_move_node);
    pre_move_node = move_node;
    move_node = node;
}

void Sequencer::command_move_interrupt_node(const std::string node){
    auto msg_move_interrupt_node = std::make_shared<std_msgs::msg::String>();
    msg_move_interrupt_node->data = node;
    _publisher_move_interrupt_node->publish(*msg_move_interrupt_node);
    pre_interrupt_node = interrupt_node;
    interrupt_node = node;
}

 void Sequencer::command_ball_tracking(){
    if(ball_pose.x > 9.90 && abs(ball_pose.y) < 0.80){
        float L = 0.296f;//m
        auto msg_coord_tracking = std::make_shared<geometry_msgs::msg::Vector3>();
        msg_coord_tracking->x = ball_pose.x - L*cos(self_pose.z);
        msg_coord_tracking->y = ball_pose.y - L*sin(self_pose.z);
        msg_coord_tracking->z = self_pose.z;
        _publisher_coord_tracking->publish(*msg_coord_tracking);
        move_progress = false;
        kado_flag = true;
        RCLCPP_INFO(get_logger(),"aaaaa bx:%f, by:%f, tx:%f, ty%f >>>>>", ball_pose.x, ball_pose.y,msg_coord_tracking->x,msg_coord_tracking->y);
    }
    else {
        float L = 0.566f;//m
        auto msg_coord_tracking = std::make_shared<geometry_msgs::msg::Vector3>();
        msg_coord_tracking->x = ball_pose.x - L*cos(self_pose.z);
        msg_coord_tracking->y = ball_pose.y - L*sin(self_pose.z);
        msg_coord_tracking->z = self_pose.z;
        _publisher_coord_tracking->publish(*msg_coord_tracking);
        move_progress = false;
        kado_flag = false;
        RCLCPP_INFO(get_logger(),"bbbbb bx:%f, by:%f, tx:%f, ty%f >>>>>", ball_pose.x, ball_pose.y,msg_coord_tracking->x,msg_coord_tracking->y);
    }
 }

 void Sequencer::command_move_autonomous(bool flag){
    auto msg_move_autonomous = std::make_shared<std_msgs::msg::Bool>();
    msg_move_autonomous->data = flag;
    _publisher_move_autonomous->publish(*msg_move_autonomous);
}

void Sequencer::command_sequence(const SEQUENCE_MODE sequence){
    pre_sequence = sequence_mode;
    sequence_mode = sequence;
    auto msg_now_sequence = std::make_shared<std_msgs::msg::String>();
    msg_now_sequence->data = sequence_list[static_cast<int>(sequence)];
    _publisher_now_sequence->publish(*msg_now_sequence);
    progress = 0;
    silo_flag = false;
    priority_num = 0;
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

void Sequencer::command_net_open(){ command_canusb_uint8(can_net_id, 0); }
void Sequencer::command_net_close(){ command_canusb_uint8(can_net_id, 1); }
void Sequencer::command_hand_suction_on(){ command_canusb_uint8(can_hand_suction_id, 0); }
void Sequencer::command_hand_suction_off(){ command_canusb_uint8(can_hand_suction_id, 1); }
void Sequencer::command_base_state(){ command_canusb_empty(can_base_state_id); }
void Sequencer::command_strage_state(bool front_ball){ 
    auto msg_canusb = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_canusb->canid = can_strage_state_id;
    msg_canusb->candlc = 8;
    uint8_t candata[8];
    float_to_bytes(candata, static_cast<float>(strage_dist[front_ball]));
    float_to_bytes(candata+4, static_cast<float>(suction_wait));
    for(int i=0; i<msg_canusb->candlc; i++) msg_canusb->candata[i] = candata[i];
    _publisher_canusb->publish(*msg_canusb);
}
void Sequencer::command_strage_state2(){ command_canusb_empty(can_strage_state2_id); }
void Sequencer::command_transfer_state(){ command_canusb_empty(can_transfer_state_id); }
void Sequencer::command_silo_state(){ command_canusb_empty(can_silo_state_id); }
void Sequencer::command_silo_state2(){ command_canusb_empty(can_silo_state2_id); }
void Sequencer::command_hand_lift_pickup(){ command_canusb_uint8(can_hand_lift_id, 2); }

void Sequencer::silo_evaluate(std::string camera[15]){
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
    int silo_level[5][5] = {};
    for(int i = 0; i < 5; i++){
        bool check = false;
        for(int j = 0; j < 11; j++){
            if(silo_data[i][0] == silo_norm[j][0] && silo_data[i][1] == silo_norm[j][1] && silo_data[i][2] == silo_norm[j][2]) {
                silo_level[i][0] = i; 
                silo_level[i][1] = std::stoi(silo_norm[j][3]);
                check = true;
                break;
            }
        }
        if(!check) {
            silo_level[i][0] = i;
            silo_level[i][1] = 10;
        }
    }
    //評価比較
    for (int i = 0; i < 4; i++) {
        for (int j = 4; j > i; j--) {
            if (silo_level[j-1][1] > silo_level[j][1]) {
                int temp0 = silo_level[j-1][0];
                silo_level[j-1][0] = silo_level[j][0];
                silo_level[j][0] = temp0;
                int temp1 = silo_level[j-1][1];
                silo_level[j-1][1] = silo_level[j][1];
                silo_level[j][1] = temp1;
            }
        }
    }
    for(int i = 0; i < 5; i++ ) silo_priority[i] = silo_level[i][0];
    silo_flag = true;
}

void Sequencer::timer(int ms){
    time = std::chrono::system_clock::now();
    check_time = ms;
}

bool Sequencer::timer(){
    if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - time).count() > check_time) return true;
    else return false;
}

bool Sequencer::check_way_ST(){
    if(way_point == "ST0" ||way_point == "ST1" ||way_point == "ST2" ||way_point == "ST3" ||way_point == "ST4" ||way_point == "ST5" ||way_point == "ST6" ||way_point == "ST7") return true;
    else return false;
}

bool Sequencer::check_way_SI(){
    if(way_point == "SI0" ||way_point == "SI1" ||way_point == "SI2" ||way_point == "SI3" ||way_point == "SI4") return true;
    else return false;
}

bool Sequencer::check_way_si(){
    if(way_point == "si0" ||way_point == "si1" ||way_point == "si2" ||way_point == "si3" ||way_point == "si4") return true;
    else return false;
}

}  // namespace sequencer