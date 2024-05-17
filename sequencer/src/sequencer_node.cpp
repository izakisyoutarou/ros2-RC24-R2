#include "sequencer/sequencer_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>

#include "utilities/can_utils.hpp"

using namespace utils;

namespace sequencer{

Sequencer::Sequencer(const rclcpp::NodeOptions &options) : Sequencer("", options) {}

Sequencer::Sequencer(const std::string &name_space, const rclcpp::NodeOptions &options)
: rclcpp::Node("sequencer_node", name_space, options),

        can_paddy_collect_id(get_parameter("canid.paddy_collect").as_int()),
        can_paddy_install_id(get_parameter("canid.paddy_install").as_int()),
        can_net_id(get_parameter("canid.net").as_int()),

        can_tof_id(get_parameter("canid.tof").as_int()),
        can_hand_lift_id(get_parameter("canid.hand_lift").as_int()),
        can_hand_fb_id(get_parameter("canid.hand_fb").as_int()),
        can_hand_wrist_id(get_parameter("canid.hand_wrist").as_int()),
        can_hand_suction_id(get_parameter("canid.suction").as_int()),

        can_base_state_id(get_parameter("canid.base_state").as_int()),
        can_strage_state_id(get_parameter("canid.strage_state").as_int()),
        can_strage_state2_id(get_parameter("canid.strage_state2").as_int()),
        can_transfer_state_id(get_parameter("canid.transfer_state").as_int()),
        can_silo_state_id(get_parameter("canid.silo_state").as_int()),
        can_silo_state2_id(get_parameter("canid.silo_state2").as_int()),

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

    _publisher_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
    _publisher_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
    _publisher_now_sequence = this->create_publisher<std_msgs::msg::String>("now_sequence", _qos);
    _publisher_move_interrupt_node = this->create_publisher<std_msgs::msg::String>("move_interrupt_node", _qos);
    _publisher_coord_tracking = this->create_publisher<geometry_msgs::msg::Vector3>("coord_tracking", _qos);

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
    // std::string A[15] ={"","B","B","","","","","R","R","","","","","B",""    };
    // silo_evaluate(A);
}

void Sequencer::callback_convergence(const controller_interface_msg::msg::Convergence::SharedPtr msg){
    
    if(!is_start) return;
    int n = 0;

    if(sequence_mode == SEQUENCE_MODE::storage){
        if(special0){
            int s = 0;
            if(special_progress == s++){
                RCLCPP_INFO(get_logger(),"_____strage3_S0_0_____");
                command_hand_suction_off();
                command_strage_state2();
                timer(500);
                special_progress++;
            }
            else if(special_progress == s++ && timer()){
                RCLCPP_INFO(get_logger(),"_____strage3_S0_1_____");
                // command_move_interrupt_node(move_node);
                if(c4_flag) command_move_interrupt_node("c6");
                else  command_move_interrupt_node("c3");
                get_suction_check = false;
                special_progress = 0;
                special0 = false;
                progress--;
                progress--;
            }
            return;
        }
        if(special1){
            int s = 0;
            if(special_progress == s++){
                RCLCPP_INFO(get_logger(),"_____strage3_S1_0_____");
                command_strage_state2();
                timer(1000);
                special_progress++;
            }
            else if(special_progress == s++ && timer()){
                RCLCPP_INFO(get_logger(),"_____strage3_S1_1_____");
                command_move_interrupt_node(move_node);
                get_suction_check = false;
                special_progress = 0;
                special1 = false;
                progress--;
            }
            return;
        }
        if(progress == n++){
            RCLCPP_INFO(get_logger(),"_____strage0_____");
            if(c4_flag || retry_flag) command_move_node("c6");
            else command_move_node("c3");
            timer(1000);
            progress++;
        }
        else if(progress == n++ && timer()){
            RCLCPP_INFO(get_logger(),"_____strage1_____");
            command_base_state();
            progress++;
        }
        else if(progress == n++ && c3orc6_flag){
            RCLCPP_INFO(get_logger(),"_____strage2_____");
            command_hand_suction_on();
            c3orc6_flag = false;
            progress++;
        }
        else if(progress == n++ && msg->arm_convergence && get_front_ball && check_way_ST()){
            RCLCPP_INFO(get_logger(),"_____strage3_____");
            // RCLCPP_INFO(get_logger(),"%d",front_ball);
            command_strage_state(front_ball); 
            get_front_ball = false;
            get_suction_check = false;
            progress++;
        }
        else if(progress == n++ && !msg->arm_convergence){
            progress++;            
        }
        else if(progress == n++ && get_suction_check && msg->arm_convergence){
            // suction_check = "M";//デバック用
            if(suction_check == "M"){
                RCLCPP_INFO(get_logger(),"_____strage4_M_____");
                command_strage_state2();
                command_sequence(SEQUENCE_MODE::silo); 
                get_suction_check = false;
            }
            else {
                RCLCPP_INFO(get_logger(),"_____strage4_P_____");
                special0 = true;
            }
            // else if(suction_check == "P") {
            //     RCLCPP_INFO(get_logger(),"_____strage4_P_____");
            //     special0 = true;
            // }
            // else if(suction_check == ""){
            //     RCLCPP_INFO(get_logger(),"_____strage4_[]_____");
            //     special1 = true;
            // }
        }
        // if(special0){
        //     int s = 0;
        //     if(special_progress == s++){
        //         RCLCPP_INFO(get_logger(),"_____strage3_S0_0_____");
        //         command_hand_suction_off();
        //         command_strage_state2();
        //         timer(2000);
        //         special_progress++;
        //     }
        //     else if(special_progress == s++ && timer()){
        //         RCLCPP_INFO(get_logger(),"_____strage3_S0_1_____");
        //         command_move_interrupt_node(move_node);
        //         get_suction_check = false;
        //         special_progress = 0;
        //         special0 = false;
        //         progress--;
        //     }
        //     return;
        // }
        // if(special1){
        //     int s = 0;
        //     if(special_progress == s++){
        //         RCLCPP_INFO(get_logger(),"_____strage3_S1_0_____");
        //         command_strage_state2();
        //         timer(1000);
        //         special_progress++;
        //     }
        //     else if(special_progress == s++ && timer()){
        //         RCLCPP_INFO(get_logger(),"_____strage3_S1_1_____");
        //         command_move_interrupt_node(move_node);
        //         get_suction_check = false;
        //         special_progress = 0;
        //         special1 = false;
        //         progress--;
        //     }
        //     return;
        // }
        // if(progress == n++){
        //     RCLCPP_INFO(get_logger(),"_____strage0_____");
        //     command_move_node("c2");
        //     timer(2000);
        //     progress++;
        // }
        // else if(progress == n++ && timer()){
        //     RCLCPP_INFO(get_logger(),"_____strage1_____");
        //     command_base_state();
        //     progress++;
        // }      
        // else if(progress == n++ && c3orc6_flag){
        //     RCLCPP_INFO(get_logger(),"_____strage2_____");
        //     c3orc6_flag = false;
        //     command_hand_suction_on();
        //     progress++;
        // }
        // else if(progress == n++ && msg->arm_convergence && get_front_ball && check_way_ST()){
        //     RCLCPP_INFO(get_logger(),"_____strage3_____");
        //     command_strage_state(front_ball); 
        //     get_front_ball = false;
        //     progress++;
        // }
        // else if(progress == n++ && get_suction_check && msg->arm_convergence){
        //     suction_check = "M";
        //     if(suction_check == "M"){
        //         RCLCPP_INFO(get_logger(),"_____strage4_M_____");
        //         command_strage_state2();
        //         command_sequence(SEQUENCE_MODE::silo); 
        //         get_suction_check = false;
        //     }
        //     else if(suction_check == "P") {
        //         RCLCPP_INFO(get_logger(),"_____strage4_P_____");
        //         special0 = true;
        //     }
        //     else if(suction_check == ""){
        //         RCLCPP_INFO(get_logger(),"_____strage4_[]_____");
        //         special1 = true;
        //     }
        // }
    }

    else if(sequence_mode == SEQUENCE_MODE::transfer){
        if(progress == n++){
            RCLCPP_INFO(get_logger(),"_____transfer0_____");
            command_move_node("ST8");
            command_net_open();
            progress++;
        }
        else if(progress == n++ && msg->net_convergence && way_point == "ST8"){
            RCLCPP_INFO(get_logger(),"_____transfer1_____");
            command_base_state();
            command_net_close();
            progress++;
        }
        else if(progress == n++  && (msg->net_convergence || tof[0])){
            RCLCPP_INFO(get_logger(),"_____transfer2_____");
            command_hand_suction_on();
            command_transfer_state();
            command_sequence(SEQUENCE_MODE::silo);
        }
    }

    else if(sequence_mode == SEQUENCE_MODE::silo){
        if(progress == n++){
            RCLCPP_INFO(get_logger(),"_____silo0_____");
            command_move_node("c1");
            progress++;
        }
        else if(progress == n++ && c1_flag && msg->arm_convergence && silo_flag){
            RCLCPP_INFO(get_logger(),"_____silo1_____");
            command_silo_state();
            c1_flag = false;
            silo_flag = false;
            priority_num = 0;
            progress++;
        }       
        else if(progress == n++ && msg->arm_convergence){
            RCLCPP_INFO(get_logger(),"_____silo2_____");
            std::string silo_node = "si" + std::to_string(silo_priority[priority_num]);
            command_move_interrupt_node(silo_node);
            progress++; 
        } 
        else if(progress == n++ && way_point ==  interrupt_node){
            RCLCPP_INFO(get_logger(),"_____silo3_____");
            RCLCPP_INFO(get_logger(),"@@@%d@@@",tof[2]);
            if(priority_num == 4){
                std::string silo_node = "SI" + std::to_string(silo_priority[priority_num]);
                command_move_interrupt_node(silo_node);
                progress++; 
            } 
            else if(tof[2]){
                priority_num++;
                progress--; 
            } 
            else {
                std::string silo_node = "SI" + std::to_string(silo_priority[priority_num]);
                command_move_interrupt_node(silo_node);
                progress++;
            }
        }
        else if(progress == n++ && check_way_SI()){
            RCLCPP_INFO(get_logger(),"_____silo4_____");
            command_hand_suction_off();
            timer(500);
            progress++;
        } 
        else if(progress == n++ && msg->arm_convergence && timer()) {
            RCLCPP_INFO(get_logger(),"_____silo5_____");
            command_silo_state2();
            if(pre_sequence == SEQUENCE_MODE::storage) command_sequence(SEQUENCE_MODE::storage);
            else if(pre_sequence == SEQUENCE_MODE::transfer) command_sequence(SEQUENCE_MODE::transfer);
        }
    }
} 

void Sequencer::callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart) {
        command_sequence(SEQUENCE_MODE::stop);
        is_start = false;
        c3orc6_flag = false;
        c1_flag = false;
        way_point = "";
        progress = 0;
        special_progress = 0;
        special0 = false;
        special1 = false;
        get_front_ball = false;
        c4_flag = false;
    }
    if(msg->initial_state == "O") retry_flag = false;
    else if(msg->initial_state == "A") retry_flag = true;
}

void Sequencer::callback_is_start(const std_msgs::msg::UInt8::SharedPtr msg){
    if(msg->data == 0) command_sequence(SEQUENCE_MODE::storage);
    else if(msg->data == 1) command_sequence(SEQUENCE_MODE::transfer);
    else if(msg->data == 2) command_sequence(SEQUENCE_MODE::collect);
    else if(msg->data == 3) command_sequence(SEQUENCE_MODE::silo);
    is_start = true;
}

void Sequencer::callback_collection_point(const std_msgs::msg::String::SharedPtr msg){
    if((!retry_flag && way_point == "c6" && msg->data == "" && sequence_mode == SEQUENCE_MODE::storage) || (retry_flag && way_point == "c3" && msg->data == "" && sequence_mode == SEQUENCE_MODE::storage)) command_sequence(SEQUENCE_MODE::transfer);
    else command_move_interrupt_node(msg->data);
    if(msg->data == "c4") c4_flag = true;

}

void Sequencer::callback_way_point(const std_msgs::msg::String::SharedPtr msg){
    way_point = msg->data;
    if((msg->data == "c3" || msg->data == "c6") && sequence_mode == SEQUENCE_MODE::storage) c3orc6_flag = true;
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
    // RCLCPP_INFO(get_logger(),"%d %d %d",tof[0],tof[1],tof[2]);
    // tof_buffer[4] = tof_buffer[3];
    // tof_buffer[3] = tof_buffer[2];
    // tof_buffer[2] = tof_buffer[1];
    // tof_buffer[1] = tof_buffer[0];
    // tof_buffer[0] = msg->candata[2];
    // int num = 0;
    // for(int i = 0; i < 5; i++) num += tof_buffer[i];
    // if(num > 0) tof[2] = true;
    // else tof[2] = false;

};

void Sequencer::callback_silo_param(const detection_interface_msg::msg::SiloParam::SharedPtr msg){
    if(sequence_mode == SEQUENCE_MODE::silo  && !silo_flag){
        std::string ball_color[15];
        for(int i = 0; i < 15; i++) ball_color[i] = msg->ball_color[i];
        silo_evaluate(ball_color);
    }
    std::cout<<msg<<std::endl;
}

void Sequencer::callback_suction_check(const std_msgs::msg::String::SharedPtr msg){
    if(msg->data == "B" || msg->data == "R") suction_check = "M";
    else suction_check = msg->data; 
    get_suction_check = true;
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

void Sequencer::command_sequence(const SEQUENCE_MODE sequence){
    pre_sequence = sequence_mode;
    sequence_mode = sequence;
    auto msg_now_sequence = std::make_shared<std_msgs::msg::String>();
    msg_now_sequence->data = sequence_list[static_cast<int>(sequence)];
    _publisher_now_sequence->publish(*msg_now_sequence);
    progress = 0;
    silo_flag = false;
    progress = 0;
    special_progress = 0;
    special0 = false;
    special1 = false;
    priority_num = 0;
    c3orc6_flag = false;
    get_front_ball = false;
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

    for(int i = 0; i < 5; i++ ) {
        silo_priority[i] = silo_level[i][0];
        // RCLCPP_INFO(get_logger(),"%d",silo_priority[i]);
    }

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