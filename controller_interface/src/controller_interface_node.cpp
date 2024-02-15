#include "controller_interface/controller_interface_node.hpp"
#include <sys/time.h>
#include <sys/types.h>
#include <chrono>
#include <iostream>
#include <future>

using namespace utils;

namespace controller_interface
{
    using std::string;

    //下記2文は共有ライブラリを書くのに必要なプログラム
    SmartphoneGamepad::SmartphoneGamepad(const rclcpp::NodeOptions &options) : SmartphoneGamepad("", options) {}
    SmartphoneGamepad::SmartphoneGamepad(const std::string &name_space, const rclcpp::NodeOptions &options): rclcpp::Node("controller_interface_node", name_space, options),
        
        //mainexecutorのyamlで設定したパラメータを設定している。
        //この場合はhigh_limit_linearクラスにDEL_MAX=poslimit,vel,acc,decのパラメータを引数として持たせている。
        //as_doubleは引用先のパラメータの型を示している。
        high_limit_linear(DBL_MAX,
        get_parameter("high_linear_max_vel").as_double(),
        get_parameter("high_linear_max_acc").as_double(),
        get_parameter("high_linear_max_dec").as_double() ),
        //以下の2つの関数も上記と同様の処理をしている
        slow_limit_linear(DBL_MAX,
        get_parameter("slow_linear_max_vel").as_double(),
        get_parameter("slow_linear_max_acc").as_double(),
        get_parameter("slow_linear_max_dec").as_double() ),
        limit_angular(DBL_MAX,
        dtor(get_parameter("angular_max_vel").as_double()),
        dtor(get_parameter("angular_max_acc").as_double()),
        dtor(get_parameter("angular_max_dec").as_double()) ),

        joy_main(get_parameter("port.joy_main").as_int()),

        //high_linear_max_velの型をdoubleからfloatにstatic_castを用いて変換している
        //数値を保存するだけならfloatの方がdoubleより処理が速いため
        high_manual_linear_max_vel(static_cast<float>(get_parameter("high_linear_max_vel").as_double())),
        slow_manual_linear_max_vel(static_cast<float>(get_parameter("slow_linear_max_vel").as_double())),
        manual_angular_max_vel(dtor(static_cast<float>(get_parameter("angular_max_vel").as_double()))),

        //リスタートのパラメータ取得
        defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
        //緊急停止のパラメータを取得
        defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool()),
        //自動化のパラメータを取得
        defalt_move_autonomous_flag(get_parameter("defalt_move_autonomous_flag").as_bool()),
        //自動射出パラメータを取得
        defalt_arm_autonomous_flag(get_parameter("defalt_arm_autonomous_flag").as_bool()),
        //低速モードのパラメータを取得
        defalt_slow_speed_flag(get_parameter("defalt_slow_speed_flag").as_bool()),
        //ボールの色情報を取得
        defalt_color_information_flag(get_parameter("defalt_color_information_flag").as_bool()),

        //足回りが目標の値まで移動して停止した状態を保存するための変数
        //要するに収束の確認
        defalt_spline_convergence(get_parameter("defalt_spline_convergence").as_bool()),
        defalt_arm_calculator_convergence(get_parameter("defalt_arm_calculator_convergence").as_bool()),
        //射出のyaw軸が目標の値まで移動して停止した状態を保存するための変数
        defalt_arm_convergence(get_parameter("defalt_arm_convergence").as_bool()),        
        //通信系
        udp_port_state(get_parameter("port.robot_state").as_int()),
        udp_port_pole(get_parameter("port.pole_share").as_int()),
        udp_port_spline_state(get_parameter("port.spline_state").as_int()),

        //can通信とは基盤に刺さっている2つの通信専用のピンの電位差で通信する方式
        //canidの取得
        can_emergency_id(get_parameter("canid.emergency").as_int()),
        can_heartbeat_id(get_parameter("canid.heartbeat").as_int()),
        can_restart_id(get_parameter("canid.restart").as_int()),
        can_calibrate_id(get_parameter("canid.calibrate").as_int()),
        can_reset_id(get_parameter("canid.reset").as_int()),
        can_linear_id(get_parameter("canid.linear").as_int()),
        can_angular_id(get_parameter("canid.angular").as_int()),
        can_steer_reset_id(get_parameter("canid.steer_reset").as_int()),
        can_paddy_collect_id(get_parameter("canid.paddy_collect").as_int()),
        can_paddy_install_id(get_parameter("canid.paddy_install").as_int()),
        can_main_button_id(get_parameter("canid.main_button").as_int()),

        //ipアドレスの取得
        r1_pc(get_parameter("ip.r1_pc").as_string()),
        r2_pc(get_parameter("ip.r2_pc").as_string()),

        //回収、射出機構のはじめの位置の値を取得
        initial_pickup_state(get_parameter("initial_pickup_state").as_string()),
        initial_inject_state(get_parameter("initial_inject_state").as_string())

        {
            //収束の状態を確認するための周期
            const auto heartbeat_ms = this->get_parameter("heartbeat_ms").as_int();
            const auto convergence_ms = this->get_parameter("convergence_ms").as_int();
            const auto base_state_communication_ms = this->get_parameter("base_state_communication_ms").as_int();

            //hppファイルでオブジェクト化したpublisherとsubscriberの設定
            //controller_mainからsub
            _sub_main_pad = this->create_subscription<std_msgs::msg::String>(
                "main_pad",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main_pad, this, std::placeholders::_1)
            );

            _sub_screen_pad = this->create_subscription<std_msgs::msg::String>(
                "SCRN_info",
                _qos,
                std::bind(&SmartphoneGamepad::callback_screen_pad, this, std::placeholders::_1)
            );

            _sub_initial_state = this->create_subscription<std_msgs::msg::String>(
                "initial_state",
                _qos,
                std::bind(&SmartphoneGamepad::callback_initial_state, this, std::placeholders::_1)
            );

            //controller_subからsub
            _sub_pad_sub = this->create_subscription<std_msgs::msg::String>(
                "sub_pad",
                _qos,
                std::bind(&SmartphoneGamepad::callback_sub_pad, this, std::placeholders::_1)
            );

            //mainからsub
            _sub_main_arm_possible = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_202",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main, this, std::placeholders::_1)
            );

            //spline_pidからsub
            _sub_spline = this->create_subscription<std_msgs::msg::Bool>(
                "is_move_tracking",
                _qos,
                std::bind(&SmartphoneGamepad::callback_spline, this, std::placeholders::_1)
            );

            //canusbへpub
            _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            //armへpub
            _pub_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("base_control",_qos);
            _pub_convergence = this->create_publisher<controller_interface_msg::msg::Convergence>("convergence" , _qos);
            _pub_color_ball_R2 = this->create_publisher<controller_interface_msg::msg::Colorball>("color_information_R2", _qos);
            _pub_arm = this->create_publisher<std_msgs::msg::Bool>("arm_info", _qos);
            //sprine_pid
            _pub_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
            //gazebo用のpub
            _pub_gazebo = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);

            //sequenserへ
            _pub_initial_sequense = this->create_publisher<std_msgs::msg::String>("initial_sequense", _qos);

            // _pub_initial_state = this->create_publisher<std_msgs::msg::String>("initial_state_unity", _qos);
            // _pub_base_restart = this->create_publisher<std_msgs::msg::Bool>("restart_unity", _qos);
            // _pub_base_emergency = this->create_publisher<std_msgs::msg::Bool>("emergency_unity", _qos);
            // _pub_move_auto = this->create_publisher<std_msgs::msg::Bool>("move_autonomous_unity", _qos);
            // _pub_base_arm = this->create_publisher<std_msgs::msg::Bool>("arm_autonomous_unity", _qos);
            // _pub_base_state_communication = this->create_publisher<std_msgs::msg::Empty>("state_communication_unity" , _qos);

            // _pub_con_spline = this->create_publisher<std_msgs::msg::Bool>("spline_convergence_unity", _qos);
            // _pub_con_colcurator = this->create_publisher<std_msgs::msg::Bool>("arm_calcurator_unity", _qos);
            // _pub_con_arm = this->create_publisher<std_msgs::msg::Bool>("arm_convergence_unity", _qos);

            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();            msg_base_control->is_restart = defalt_restart_flag;
            msg_base_control->is_emergency = defalt_emergency_flag;
            msg_base_control->is_move_autonomous = defalt_move_autonomous_flag;
            msg_base_control->is_arm_autonomous = defalt_arm_autonomous_flag;
            msg_base_control->is_slow_speed = defalt_slow_speed_flag;
            msg_base_control->initial_state = "O";
            _pub_base_control->publish(*msg_base_control);

            //armの初期情報
            auto msg_arm_con = std::make_shared<std_msgs::msg::Bool>();
            msg_arm_con->data = arm_flag;
            _pub_arm ->publish(*msg_arm_con);

            auto msg_unity_initial_state = std::make_shared<std_msgs::msg::String>();
            msg_unity_initial_state->data = initial_state;
            _pub_initial_state->publish(*msg_unity_initial_state);

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = can_emergency_id;
            msg_emergency->candlc = 1;
            msg_emergency->candata[0] = defalt_emergency_flag;
            _pub_canusb->publish(*msg_emergency);

            auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            msg_convergence->spline_convergence = defalt_spline_convergence;
            msg_convergence->arm_calculator = defalt_arm_calculator_convergence;
            msg_convergence->arm = defalt_arm_convergence;
            _pub_convergence->publish(*msg_convergence);

            auto msg_colorball_info = std::make_shared<controller_interface_msg::msg::Colorball>();

            msg_colorball_info->color_info[0] = defalt_color_information_flag;
            msg_colorball_info->color_info[1] = defalt_color_information_flag;
            msg_colorball_info->color_info[2] = defalt_color_information_flag;
            msg_colorball_info->color_info[3] = defalt_color_information_flag;
            msg_colorball_info->color_info[4] = defalt_color_information_flag;
            msg_colorball_info->color_info[5] = defalt_color_information_flag;
            msg_colorball_info->color_info[6] = defalt_color_information_flag;
            msg_colorball_info->color_info[7] = defalt_color_information_flag;
            msg_colorball_info->color_info[8] = defalt_color_information_flag;
            msg_colorball_info->color_info[9] = defalt_color_information_flag;
            msg_colorball_info->color_info[10] = defalt_color_information_flag;
            msg_colorball_info->color_info[11] = defalt_color_information_flag;
            msg_colorball_info->color_info[12] = defalt_color_information_flag;
            msg_colorball_info->color_info[13] = defalt_color_information_flag;
            msg_colorball_info->color_info[14] = defalt_color_information_flag;
            msg_colorball_info->color_info[15] = defalt_color_information_flag;
            _pub_color_ball_R2->publish(*msg_colorball_info);

            //ハートビート
            _pub_heartbeat = this->create_wall_timer(
                std::chrono::milliseconds(heartbeat_ms),
                [this] {
                    auto msg_heartbeat = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_heartbeat->canid = can_heartbeat_id;
                    msg_heartbeat->candlc = 0;
                    _pub_canusb->publish(*msg_heartbeat);
                }
            );
            //スマホコントローラとの通信状況を確認
            _pub_state_communication_timer = create_wall_timer(
                std::chrono::milliseconds(base_state_communication_ms),
                [this] {
                    auto msg_base_state_communication = std::make_shared<std_msgs::msg::Empty>();
                    _pub_base_state_communication->publish(*msg_base_state_communication);
                }
            );

            //convergence
            _pub_timer_convergence = this->create_wall_timer(
                std::chrono::milliseconds(convergence_ms),
                [this] {
                    auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
                    msg_convergence->spline_convergence = is_spline_convergence;                   msg_convergence->arm = is_arm_convergence;
                    _pub_convergence->publish(*msg_convergence);
                }
            );

            _socket_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("interval_ms").as_int()),
                [this] { _recv_callback(); }
            );
            
            _start_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("start_ms").as_int()),
                [this] {
                    if(start_flag)
                    {
                        auto initial_sequense_injection = std::make_shared<std_msgs::msg::String>();
                        initial_sequense_injection->data = initial_inject_state;
                        _pub_initial_sequense->publish(*initial_sequense_injection);
                    }
                }
            );

            //計画機にリミットを設定する
            high_velPlanner_linear_x.limit(high_limit_linear);
            high_velPlanner_linear_y.limit(high_limit_linear);

            slow_velPlanner_linear_x.limit(slow_limit_linear);
            slow_velPlanner_linear_y.limit(slow_limit_linear);

            velPlanner_angular_z.limit(limit_angular);
        }

        void SmartphoneGamepad::callback_main_pad(const std_msgs::msg::String::SharedPtr msg)
        {
            //リスタート
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = can_restart_id;
            msg_restart->candlc = 0;

            //緊急停止
            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = can_emergency_id;
            msg_emergency->candlc = 1;

            //ボタン
            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = can_main_button_id;
            msg_btn->candlc = 8;

            uint8_t _candata_btn[8];
            bool robotcontrol_flag = false;
            bool flag_restart = false;

            if(msg->data == "g"){
                RCLCPP_INFO(this->get_logger(), "g");
                robotcontrol_flag = true;
                is_emergency = true;
                is_reset = false;  
                start_flag = false;            
            }

            if(msg->data == "s")
            {
                RCLCPP_INFO(this->get_logger(), "s");
                robotcontrol_flag = true;
                flag_restart = true;
                is_emergency = false;
                is_reset = true;
                start_flag = true;
                is_move_autonomous = defalt_move_autonomous_flag;
                is_arm_autonomous = defalt_arm_autonomous_flag;
                is_slow_speed = defalt_slow_speed_flag;
                initial_state = "O";

                is_spline_convergence = defalt_spline_convergence;
                is_arm_convergence = defalt_arm_convergence;
            }

            //ステアリセット
            if(msg->data == "up"){
                RCLCPP_INFO(this->get_logger(), "up");
                auto msg_steer_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                msg_steer_reset->canid = can_steer_reset_id;
                msg_steer_reset->candlc = 0;
                _pub_canusb->publish(*msg_steer_reset);
                //gamebtn.steer_reset(can_steer_reset_id,_pub_canusb);
            }

            //キャリブレーション
            if(msg->data == "down"){
                RCLCPP_INFO(this->get_logger(), "down");
                // auto msg_calibrate = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                // msg_calibrate->canid = can_calibrate_id;
                // msg_calibrate->candlc = 0;
                // _pub_canusb->publish(*msg_calibrate);
                gamebtn.calibrate(can_calibrate_id,_pub_canusb);
            }
            //main基盤リセット
            if(msg->data == "right"){
                RCLCPP_INFO(this->get_logger(), "right");
                // auto msg_main_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                // msg_main_reset->canid = can_reset_id;
                // msg_main_reset->candlc = 1;
                // msg_main_reset->candata[0] = 0;
                // _pub_canusb->publish(*msg_main_reset);
                gamebtn.main_reset(can_reset_id,_pub_canusb);
            }

            //IO基盤リセット
            if(msg->data == "left"){
                RCLCPP_INFO(this->get_logger(), "left");
                // auto msg_io_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                // msg_io_reset->canid = can_reset_id;
                // msg_io_reset->candlc = 1;
                // msg_io_reset->candata[0] = 1;
                // _pub_canusb->publish(*msg_io_reset);
                gamebtn.io_reset(can_reset_id,_pub_canusb);
            }

            if(msg->data == "a"){
                RCLCPP_INFO(this->get_logger(), "a");
                if(is_arm_convergence){
                //     auto msg_paddy_install = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                //     msg_paddy_install->canid = can_paddy_install_id;
                //     msg_paddy_install->candlc = 0;
                //     _pub_canusb->publish(*msg_paddy_install); 
                gamebtn.paddy_install(can_paddy_install_id,_pub_canusb); 
                }   
    
            }

            if(msg->data == "b"){
                RCLCPP_INFO(this->get_logger(), "b");
                if(is_arm_convergence){
                    // auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    // msg_paddy_collect->canid = can_paddy_collect_id;
                    // msg_paddy_collect->candlc = 1;
                    // msg_paddy_collect->candata[0] = 0;
                    // _pub_canusb->publish(*msg_paddy_collect);
                    gamebtn.paddy_collect_1(can_paddy_collect_id,_pub_canusb);
                }
            }
            
            if(msg->data == "x"){
                RCLCPP_INFO(this->get_logger(), "x");
                if(is_arm_convergence){
                    // auto msg_paddy_collect = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    // msg_paddy_collect->canid = can_paddy_collect_id;
                    // msg_paddy_collect->candlc = 1;
                    // msg_paddy_collect->candata[0] = 1;
                    // _pub_canusb->publish(*msg_paddy_Scollect);
                    gamebtn.paddy_collect_2(can_paddy_collect_id,_pub_canusb);
                }
            }
            
            if(msg->data == "y"){
                RCLCPP_INFO(this->get_logger(), "y");
            }

            if(msg->data == "r1"){
                RCLCPP_INFO(this->get_logger(), "r1");
            }

            //r2で低速モートのonoff。トグル。
            if(msg->data == "r2"){
                RCLCPP_INFO(this->get_logger(), "r2");
                robotcontrol_flag = true;
                if(is_slow_speed == true ){
                    is_slow_speed = false;
                }else{
                    is_slow_speed = true;
                }
            }

            //r3は足回りの手自動の切り替え。is_move_autonomousを使って、トグルになるようにしてる。R1の上物からもらう必要はない。
            if(msg->data == "r3"){
                RCLCPP_INFO(this->get_logger(), "r3");
                robotcontrol_flag = true;
                if(is_move_autonomous == false){
                    is_move_autonomous = true;
                    is_arm_autonomous = true;
                }
                else{
                    is_move_autonomous = false;
                    is_arm_autonomous = false;
                }
            }

            if(msg->data == "l1"){
                RCLCPP_INFO(this->get_logger(), "l1");
            }

            if(msg->data == "l2"){
                RCLCPP_INFO(this->get_logger(), "l2");
            }

            if(msg->data == "l3"){
                auto initial_sequense_pickup = std::make_shared<std_msgs::msg::String>();
                initial_sequense_pickup->data = initial_pickup_state;
                _pub_initial_sequense->publish(*initial_sequense_pickup);
            }

            //リセットボタンを押しているか確認する
            is_reset = msg->data == "s";

            //base_controlへ代入
            msg_base_control.is_restart = is_reset;
            msg_base_control.is_emergency = is_emergency;
            msg_base_control.is_move_autonomous = is_move_autonomous;
            msg_base_control.is_arm_autonomous = is_arm_autonomous;
            msg_base_control.is_slow_speed = is_slow_speed;
            msg_base_control.initial_state = initial_state;
            
            for(int i=0; i<msg_btn->candlc; i++){
                msg_btn->candata[i] = _candata_btn[i];
            }

            //どれか１つのボタンを押すとすべてのボタン情報がpublishされる
            if( a == true ||b == true ||y == true ||x == true ||right == true ||down == true ||left == true ||up == true )
            {
                _pub_canusb->publish(*msg_btn);
            }
            msg_emergency->candata[0] = is_emergency;
            
            if(msg->data=="g")
            {
                _pub_canusb->publish(*msg_emergency);
            }
            if(robotcontrol_flag == true)
            {
                _pub_base_control->publish(msg_base_control);
            }
            if(msg->data == "s")
            {
                _pub_canusb->publish(*msg_restart);
                _pub_canusb->publish(*msg_emergency);

            }
        }

        void SmartphoneGamepad::callback_screen_pad(const std_msgs::msg::String::SharedPtr msg){
            auto msg_move_node = std::make_shared<std_msgs::msg::String>();
            if(msg->data == "O"){
                msg_move_node->data = "O";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "A"){
                msg_move_node->data = "A";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "c1"){
                msg_move_node->data = "c1";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "SI0"){
                msg_move_node->data = "SI0";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "SI1"){
                msg_move_node->data = "SI1";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "SI2"){
                msg_move_node->data = "SI2";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "SI3"){
                msg_move_node->data = "SI3";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "SI4"){
                msg_move_node->data = "SI4";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "ST0"){
                msg_move_node->data = "ST0";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "ST1"){
                msg_move_node->data = "ST1";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "ST2"){
                msg_move_node->data = "ST2";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "ST3"){
                msg_move_node->data = "ST3";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "ST4"){
                msg_move_node->data = "ST4";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "ST5"){
                msg_move_node->data = "ST5";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "ST6"){
                msg_move_node->data = "ST6";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "ST7"){
                msg_move_node->data = "ST7";
                _pub_move_node->publish(*msg_move_node);
            }
            if(msg->data == "ST8"){
                msg_move_node->data = "ST8";
                _pub_move_node->publish(*msg_move_node);
            }   
        }

        void SmartphoneGamepad::callback_sub_pad(const std_msgs::msg::String::SharedPtr msg){
            auto msg_unity_sub_control = std::make_shared<std_msgs::msg::Bool>();
            int colordlc = 16;
            bool color_data[16];

            if(msg->data == "A_red"){
                RCLCPP_INFO(this->get_logger(), "color_red_A");
                color_data[0] = true;
                msg_colorball_info.color_info[0] = color_data[0];
            }
            else if(msg->data == "A_purple"){
                color_data[0] = false;
                RCLCPP_INFO(this->get_logger(), "color_purple_A");
                msg_colorball_info.color_info[0] = color_data[0];
            }
            if(msg->data == "B_red"){
                color_data[1] = true;
                msg_colorball_info.color_info[1] = color_data[1];
            }
            else if(msg->data == "B_purple"){
                color_data[1] = false;
                msg_colorball_info.color_info[1] = color_data[1];
            }
            if(msg->data == "C_red"){
                color_data[2] = true;
                msg_colorball_info.color_info[2] = color_data[2];
            }
            else if(msg->data == "C_purple"){
                color_data[2] = false;
                msg_colorball_info.color_info[2] = color_data[2];
            }
            if(msg->data == "D_red"){
                color_data[3] = true;
                msg_colorball_info.color_info[3] = color_data[3];
            }
            else if(msg->data == "D_purple"){
                color_data[3] = false;
                msg_colorball_info.color_info[3] = color_data[3];
            }
            if(msg->data == "E_red"){
                color_data[4] = true;
                msg_colorball_info.color_info[4] = color_data[4];
            }
            else if(msg->data == "E_purple"){
                color_data[4] = false;
                msg_colorball_info.color_info[4] = color_data[4];
            }
            if(msg->data == "F_red"){
                color_data[5] = true;
                msg_colorball_info.color_info[5] = color_data[5];
            }
            else if(msg->data == "F_purple"){
                color_data[5] = false;
                msg_colorball_info.color_info[5] = color_data[5];
            }
            if(msg->data == "G_red"){
                color_data[6] = true;
                msg_colorball_info.color_info[6] = color_data[6];
            }
            else if(msg->data == "G_purple"){
                color_data[6] = false;
                msg_colorball_info.color_info[6] = color_data[6];
            }
            if(msg->data == "H_red"){
                color_data[7] = true;
                msg_colorball_info.color_info[7] = color_data[7];
            }
            else if(msg->data == "H_purple"){
                color_data[7] = false;
                msg_colorball_info.color_info[7] = color_data[7];
            }
            if(msg->data == "I_red"){
                color_data[8] = true;
                msg_colorball_info.color_info[8] = color_data[8];
            }
            else if(msg->data == "I_purple"){
                color_data[8] = false;
                msg_colorball_info.color_info[8] = color_data[8];
            }
            if(msg->data == "J_red"){
                color_data[9] = true;
                msg_colorball_info.color_info[9] = color_data[9];
            }
            else if(msg->data == "J_purple"){
                color_data[9] = false;
                msg_colorball_info.color_info[9] = color_data[9];
            }
            if(msg->data == "K_red"){
                color_data[10] = true;
                msg_colorball_info.color_info[10] = color_data[10];
            }
            else if(msg->data == "K_purple"){
                color_data[10] = false;
                msg_colorball_info.color_info[10] = color_data[10];
            }
            if(msg->data == "L_red"){
                color_data[11] = true;
                msg_colorball_info.color_info[11] = color_data[11];
            }
            else if(msg->data == "L_purple"){
                color_data[11] = false;
                msg_colorball_info.color_info[11] = color_data[11];
            }
            if(msg->data == "M_red"){
                color_data[12] = true;
                msg_colorball_info.color_info[12] = color_data[12];
            }
            else if(msg->data == "M_purple"){
                color_data[12] = false;
                msg_colorball_info.color_info[12] = color_data[12];
            }
            if(msg->data == "N_red"){
                color_data[13] = true;
                msg_colorball_info.color_info[13] = color_data[13];
            }
            else if(msg->data == "N_purple"){
                color_data[13] = false;
                msg_colorball_info.color_info[13] = color_data[13];
            }
            if(msg->data == "O_red"){
                color_data[14] = true;
                msg_colorball_info.color_info[14] = color_data[14];
            }
            else if(msg->data == "O_purple"){
                color_data[14] = false;
                msg_colorball_info.color_info[14] = color_data[14];
            }
            if(msg->data == "P_red"){
                color_data[15] = true;
                msg_colorball_info.color_info[15] = color_data[15];
            }
            else if(msg->data == "P_purple"){
                color_data[15] = false;
                msg_colorball_info.color_info[15] = color_data[15];
            }
            // for(int k=0; k<colordlc;k++){
            //     msg_colorball_info.color_info[k] = color_data[k];
            // }
            
            if(msg->data == "Btn_info_msg"){
                RCLCPP_INFO(this->get_logger(), "color_info_all");
                _pub_color_ball_R2->publish(msg_colorball_info);
            }

        }

            //コントローラからスタート地点情報をsubscribe
        void SmartphoneGamepad::callback_initial_state(const std_msgs::msg::String::SharedPtr msg)
        {
            initial_state = msg->data[0];
            auto msg_unity_initial_state = std::make_shared<std_msgs::msg::String>();
            msg_unity_initial_state->data = initial_state;
            _pub_initial_state->publish(*msg_unity_initial_state);
        }
        
        //コントローラから射出情報をsubsclib
        void SmartphoneGamepad::callback_main(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            ///mainから射出可能司令のsub。上物の収束状況。
            //RCLCPP_INFO(this->get_logger(), "can_rx_202");
            is_arm_convergence = static_cast<bool>(msg->candata[0]);
        }

        //splineからの情報をsubsclib
        void SmartphoneGamepad::callback_spline(const std_msgs::msg::Bool::SharedPtr msg)
        {
            is_spline_convergence = msg->data;
        }
        //スティックの値をUDP通信でsubscribしている
        void SmartphoneGamepad::_recv_callback()
        {
            if(joy_main.is_recved())
            {
                //メモリの使用量を減らすためunsignedを使う
                unsigned char data[16];
                //sizeof関数でdataのメモリを取得
                _recv_joy_main(joy_main.data(data, sizeof(data)));
            }
        }

        //ジョイスティック
        void SmartphoneGamepad::_recv_joy_main(const unsigned char data[16])
        {
            float values[4];
            //メモリをコピー
            memcpy(values, data, sizeof(float)*4);
            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = can_linear_id;
            msg_linear->candlc = 8;
            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = can_angular_id;
            msg_angular->candlc = 4;
            auto msg_gazebo = std::make_shared<geometry_msgs::msg::Twist>();

            bool flag_move_autonomous = false;
            
            uint8_t _candata_joy[8];
            //手動モード
            if(is_move_autonomous == false)
            {
                //低速モード
                if(is_slow_speed == true)
                {
                    slow_velPlanner_linear_x.vel(static_cast<double>(values[1]));//unityとロボットにおける。xとyが違うので逆にしている。
                    slow_velPlanner_linear_y.vel(static_cast<double>(-values[0]));
                    velPlanner_angular_z.vel(static_cast<double>(-values[2]));
                    slow_velPlanner_linear_x.cycle();
                    slow_velPlanner_linear_y.cycle();
                    velPlanner_angular_z.cycle();
                    //floatからバイト(メモリ)に変換
                    float_to_bytes(_candata_joy, static_cast<float>(-slow_velPlanner_linear_x.vel()) * slow_manual_linear_max_vel);
                    float_to_bytes(_candata_joy+4, static_cast<float>(-slow_velPlanner_linear_y.vel()) * slow_manual_linear_max_vel);
                    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(-velPlanner_angular_z.vel()) * manual_angular_max_vel);
                    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];
                    
                    msg_gazebo->linear.x = slow_velPlanner_linear_x.vel();
                    msg_gazebo->linear.y = slow_velPlanner_linear_y.vel();
                    msg_gazebo->angular.z = velPlanner_angular_z.vel();
                    _pub_gazebo->publish(*msg_gazebo);
                    RCLCPP_INFO(this->get_logger(), "%f",slow_velPlanner_linear_y.vel());
                    RCLCPP_INFO(this->get_logger(), "%f",slow_velPlanner_linear_x.vel());
                    RCLCPP_INFO(this->get_logger(),"%f",velPlanner_angular_z.vel());
                    
                }
                //高速モードのとき
                else
                {
                    high_velPlanner_linear_x.vel(static_cast<double>(values[1]));//unityとロボットにおける。xとyが違うので逆にしている。
                    high_velPlanner_linear_y.vel(static_cast<double>(-values[0]));
                    velPlanner_angular_z.vel(static_cast<double>(-values[2]));

                    high_velPlanner_linear_x.cycle();
                    high_velPlanner_linear_y.cycle();
                    velPlanner_angular_z.cycle();

                    float_to_bytes(_candata_joy, static_cast<float>(-high_velPlanner_linear_x.vel()) * high_manual_linear_max_vel);
                    float_to_bytes(_candata_joy+4, static_cast<float>(-high_velPlanner_linear_y.vel()) * high_manual_linear_max_vel);
                    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(-velPlanner_angular_z.vel()) * manual_angular_max_vel);
                    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                    msg_gazebo->linear.x = high_velPlanner_linear_x.vel();
                    msg_gazebo->linear.y = high_velPlanner_linear_y.vel();
                    msg_gazebo->angular.z = velPlanner_angular_z.vel();
                    _pub_gazebo->publish(*msg_gazebo);
                }
                _pub_canusb->publish(*msg_linear);
                _pub_canusb->publish(*msg_angular);
                _pub_gazebo->publish(*msg_gazebo);
            }
        }

}