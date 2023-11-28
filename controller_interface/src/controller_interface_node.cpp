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
        can_linear_id(get_parameter("canid.linear").as_int()),
        can_angular_id(get_parameter("canid.angular").as_int()),
        can_main_button_id(get_parameter("canid.main_digital_button").as_int()),
        can_sub_button_id(get_parameter("canid.sub_digital_button").as_int()),
        can_arm_silo_id(get_parameter("canid.arm_silo").as_int()),
        can_arm_collection_id(get_parameter("canid.arm_collection").as_int()),

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

            //hppファイルでオブジェクト化したpublisherとsubscriberの設定
            //controller_mainからsub
            _sub_main_pad = this->create_subscription<std_msgs::msg::String>(
                "R2_main_pad",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main_pad, this, std::placeholders::_1)
            );

            _sub_screen_pad = this->create_subscription<std_msgs::msg::String>(
                "R2SCRN_info",
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
                "R2_sub_pad",
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

            //arm_param_calculatorからsub
            _sub_arm_calculator = this->create_subscription<std_msgs::msg::Bool>(
                "is_arm_calculator_convergenced",
                _qos,
                std::bind(&SmartphoneGamepad::callback_arm_calculator, this, std::placeholders::_1)
            );

            //canusbへpub
            //txは送信でrxは受信
            _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            //armへpub
            //各nodeへ共有。
            _pub_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("base_control",_qos);
            _pub_convergence = this->create_publisher<controller_interface_msg::msg::Convergence>("convergence" , _qos);
            _pub_color_ball_R2 = this->create_publisher<controller_interface_msg::msg::Colorball>("color_information_R2", _qos);
            _pub_arm = this->create_publisher<std_msgs::msg::Bool>("arm_info", _qos);
            //sprine_pid
            _pub_move_node = this->create_publisher<std_msgs::msg::String>("move_node", _qos);
            //gazebo用のpub
            _pub_gazebo = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", _qos);

            _pub_initial_state = this->create_publisher<std_msgs::msg::String>("R2_initial_state_unity", _qos);
            _pub_base_restart = this->create_publisher<std_msgs::msg::Bool>("R2_restart_unity", _qos);
            _pub_base_emergency = this->create_publisher<std_msgs::msg::Bool>("R2_emergency_unity", _qos);
            _pub_move_auto = this->create_publisher<std_msgs::msg::Bool>("R2_move_autonomous_unity", _qos);
            _pub_base_arm = this->create_publisher<std_msgs::msg::Bool>("R2_arm_autonomous_unity", _qos);

            _pub_con_spline = this->create_publisher<std_msgs::msg::Bool>("R2_spline_convergence_unity", _qos);
            _pub_con_colcurator = this->create_publisher<std_msgs::msg::Bool>("R2_arm_calcurator_unity", _qos);
            _pub_con_arm = this->create_publisher<std_msgs::msg::Bool>("R2_arm_convergence_unity", _qos);

            //デフォルト値をpub.。各種、boolに初期値を代入。
            //base_controlのmsgを宣言
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            //get_parametorで取得したパラメータをrc23pkgsのmsgに格納
            msg_base_control->is_restart = defalt_restart_flag;
            msg_base_control->is_emergency = defalt_emergency_flag;
            msg_base_control->is_move_autonomous = defalt_move_autonomous_flag;
            msg_base_control->is_arm_autonomous = defalt_arm_autonomous_flag;
            msg_base_control->is_slow_speed = defalt_slow_speed_flag;
            msg_base_control->initial_state = "O";
            //hppファイルに宣言されたbool型の変数に格納
            //同じ変数名が2つあるのでアロー演算子を用いる
            this->is_reset = defalt_restart_flag;
            this->is_emergency = defalt_emergency_flag;
            this->is_move_autonomous = defalt_move_autonomous_flag;
            this->is_arm_autonomous = defalt_arm_autonomous_flag;
            this->is_slow_speed = defalt_slow_speed_flag;
            this->initial_state = "O";
            //格納された値をpublish
            _pub_base_control->publish(*msg_base_control);

            //armの初期情報
            auto msg_arm_con = std::make_shared<std_msgs::msg::Bool>();
            msg_arm_con->data = arm_flag;
            _pub_arm ->publish(*msg_arm_con);

            auto msg_unity_initial_state = std::make_shared<std_msgs::msg::String>();
            msg_unity_initial_state->data = initial_state;
            _pub_initial_state->publish(*msg_unity_initial_state);

            auto msg_unity_control = std::make_shared<std_msgs::msg::Bool>();
            msg_unity_control->data = is_reset;
            _pub_base_restart->publish(*msg_unity_control);

            msg_unity_control->data = is_emergency;
            _pub_base_emergency->publish(*msg_unity_control);

            msg_unity_control->data = is_move_autonomous;
            _pub_move_auto->publish(*msg_unity_control);

            msg_unity_control->data = is_arm_autonomous;
            _pub_base_arm->publish(*msg_unity_control);

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            //get_parametorで取得したパラメータをrc23pkgsのmsgに格納
            msg_emergency->canid = can_emergency_id;
            //trueかfalseなので使用するバイトは1つ
            msg_emergency->candlc = 1;
            msg_emergency->candata[0] = defalt_emergency_flag;
            _pub_canusb->publish(*msg_emergency);

            //収束を確認するmsgの宣言
            auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            //get_parametorで取得したパラメータをrc23pkgsのmsgに格納
            msg_convergence->spline_convergence = defalt_spline_convergence;
            msg_convergence->arm_calculator = defalt_arm_calculator_convergence;
            msg_convergence->arm = defalt_arm_convergence;

            this->spline_convergence = defalt_spline_convergence;
            this->arm_calculator = defalt_arm_calculator_convergence;
            this->arm = defalt_arm_convergence;

            _pub_convergence->publish(*msg_convergence);

            msg_unity_control->data = spline_convergence;
            _pub_con_spline->publish(*msg_unity_control);

            msg_unity_control->data = arm_calculator;
            _pub_con_colcurator->publish(*msg_unity_control);

            msg_unity_control->data = arm;
            _pub_con_arm->publish(*msg_unity_control);

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
            //コントローラの鼓動
            //一定周期で処理をしている。この場合は100ms間隔で処理をしている
            _pub_heartbeat = this->create_wall_timer(
                std::chrono::milliseconds(heartbeat_ms),
                [this] {
                    auto msg_heartbeat = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    //get_parametorで取得したパラメータをrc23pkgsのmsgに格納
                    msg_heartbeat->canid = can_heartbeat_id;
                    msg_heartbeat->candlc = 0;
                    _pub_canusb->publish(*msg_heartbeat);
                }
            );

            //convergence
            //収束状況
            //一定周期で処理をしている。この場合は100ms間隔で処理をしている
            _pub_timer_convergence = this->create_wall_timer(
                std::chrono::milliseconds(convergence_ms),
                [this] {
                    auto msg_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
                    //get_parametorで取得したパラメータをrc23pkgsのmsgに格納
                    msg_convergence->spline_convergence = is_spline_convergence;
                    msg_convergence->arm_calculator = is_arm_calculator_convergence;
                    msg_convergence->arm = is_arm_convergence;
                    
                    _pub_convergence->publish(*msg_convergence);
                }
            );

            //一定周期で処理をしている。この場合は50ms間隔で処理をしている
            //コントローラのデータを一定周期で届いているか確認する
            //UDP通信特有の書き方？
            _socket_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("interval_ms").as_int()),
                [this] { _recv_callback(); }
            );
            //一定周期で処理をしている。この場合は3000ms間隔で処理をしている
            //
            _start_timer = this->create_wall_timer(
                std::chrono::milliseconds(this->get_parameter("start_ms").as_int()),
                [this] {
                    if(start_flag)
                    {
                        const string initial_inject_state_with_null = initial_inject_state + '\0';
                        //c_strがポインタを返すためアスタリスクをつける
                        const char* char_ptr2 = initial_inject_state_with_null.c_str();
                        //reinterpret_castでポインタ型の変換
                        //char_ptr2をconst unsigned charに置き換える
                        const unsigned char* inject = reinterpret_cast<const unsigned char*>(char_ptr2);
                        //commandクラスのudp通信で一番最初に回収するデータをコントローラーに送り、コントローラ側で処理が行われる
                        command.state_num_R2(inject, r2_pc,udp_port_state);
                        start_flag = false;
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
            //リスタートの処理
            //msg_restartにリスタートする際のcanidとcandlcのパラメータを格納
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = can_restart_id;
            msg_restart->candlc = 0;

            //緊急停止の処理
            //msg_emergencyにリスタートする際のcanidとcandlcのパラメータを格納
            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = can_emergency_id;
            msg_emergency->candlc = 1;

            //ボタンの処理
            //msg_btnにリスタートする際のcanidとcandlcのパラメータを格納
            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = can_main_button_id;
            msg_btn->candlc = 8;

            uint8_t _candata_btn[8];
            //base_control(手自動、緊急、リスタート)が押されたらpubする
            //robotcontrol_flagはtrueのときpublishできる
            bool robotcontrol_flag = false;
            //resertがtureをpubした後にfalseをpubする
            bool flag_restart = false;

            auto msg_arm_move = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_arm_move->canid = can_arm_collection_id;
            msg_arm_move->candlc = 1;

            //rightで射出機構の停止
            if(msg->data == "right"){
                RCLCPP_INFO(this->get_logger(), "right");
                is_arm_mech_stop_m = true;
                msg_arm_move->candata[0] = false;
                _pub_canusb->publish(*msg_arm_move);
            }

            //up,downでアームがボールを回収する位置を決める
            if(msg->data == "left")
            {
                if(is_arm_convergence){
                    auto msg_arm_con = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_arm_con->canid = can_arm_silo_id;
                    msg_arm_con->candlc = 0;
                    _pub_canusb->publish(*msg_arm_con);
                }
            }
            auto msg_arm = std::make_shared<std_msgs::msg::Bool>();

            if(msg->data == "up")
            {
                RCLCPP_INFO(this->get_logger(), "up");
                arm_flag = false;
                msg_arm->data = arm_flag;
                _pub_arm->publish(*msg_arm);
                msg_arm_move->candata[0] = true;
                _pub_canusb->publish(*msg_arm_move);
                is_arm_mech_stop_m = false;
            }

            if(msg->data == "down")
            {
                RCLCPP_INFO(this->get_logger(), "down");
                arm_flag = true;
                msg_arm->data = arm_flag;
                _pub_arm->publish(*msg_arm);
                msg_arm_move->candata[0] = true;
                _pub_canusb->publish(*msg_arm_move);
                is_arm_mech_stop_m = false;
            }
            //r2で低速モートのonoff。トグル。
            if(msg->data == "r2")
            {
                RCLCPP_INFO(this->get_logger(), "r2");
                robotcontrol_flag = true;
                if(is_slow_speed == true ){
                    is_slow_speed = false;
                }else{
                    is_slow_speed = true;
                }
            }
            //r3は足回りの手自動の切り替え。is_move_autonomousを使って、トグルになるようにしてる。R1の上物からもらう必要はない。
            if(msg->data == "r3")
            {
                RCLCPP_INFO(this->get_logger(), "r3");
                robotcontrol_flag = true;
                if(is_move_autonomous == false){
                    is_move_autonomous = true;
                }
                else{
                    is_move_autonomous = false;
                }
            }
            //l3でR1の状態確認
            if(msg->data == "l3")
            {
                RCLCPP_INFO(this->get_logger(), "l3");
                start_r2_main = true;
            }
            //gは緊急。is_emergencyを使って、トグルになるようにしてる。
            if(msg->data == "g"){
                RCLCPP_INFO(this->get_logger(), "g");
                robotcontrol_flag = true;
                is_emergency = true;                
            }
            //sはリスタート。緊急と手自動のboolをfalseにしてリセットしている。
            //msgがsだったときのみ以下の変数にパラメータが代入される
            if(msg->data == "s")
            {
                RCLCPP_INFO(this->get_logger(), "s");
                robotcontrol_flag = true;
                flag_restart = true;
                is_emergency = false;
                is_arm_mech_stop_m = false;
                is_move_autonomous = defalt_move_autonomous_flag;
                is_arm_autonomous = defalt_arm_autonomous_flag;
                is_slow_speed = defalt_slow_speed_flag;

                is_spline_convergence = defalt_spline_convergence;
                is_arm_calculator_convergence = defalt_arm_calculator_convergence;
                is_arm_convergence = defalt_arm_convergence;
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
            msg_base_control.is_arm_mech_stop_m = is_arm_mech_stop_m;           

            //mainへボタン情報を送る代入
            if(msg->data == "a")_candata_btn[0] = a;
            if(msg->data == "b")_candata_btn[1] = b;
            if(msg->data == "x")_candata_btn[2] = y;
            if(msg->data == "y")_candata_btn[3] = x;
            if(msg->data == "rigit")_candata_btn[4] = right;
            if(msg->data == "left")_candata_btn[5] = down;
            if(msg->data == "down")_candata_btn[6] = left;
            if(msg->data == "up")_candata_btn[7] = up;
            
            for(int i=0; i<msg_btn->candlc; i++){
                msg_btn->candata[i] = _candata_btn[i];
            }

            //どれか１つのボタンを押すとすべてのボタン情報がpublishされる
            if( a == true ||b == true ||y == true ||x == true ||right == true ||down == true ||left == true ||up == true )
            {
                _pub_canusb->publish(*msg_btn);
            }
            //l3を押すと射出情報をpublishする

            if(start_r2_main == true)
            {
                //c_strがポインタ型を返すためアス＝＝タリスクをつける
                const char* char_ptr = initial_pickup_state.c_str();
                //reinterpret_castでポインタ型の変換
                //char_ptr1をconst unsigned charに置き換える
                const unsigned char* pickup = reinterpret_cast<const unsigned char*>(char_ptr);
                //commandクラスのudp通信で一番最初に回収するデータをコントローラーに送り、コントローラ側で処理が行われる
                command.state_num_R2(pickup, r2_pc,udp_port_state);
                //同じ処理が連続で起きないようにfalseでもとの状態に戻す
                start_flag = true;
                start_r2_main = false;
            }
            msg_emergency->candata[0] = is_emergency;
            
            if(msg->data=="g")
            {
                _pub_canusb->publish(*msg_emergency);
            }
            if(robotcontrol_flag == true)
            {
                _pub_base_control->publish(msg_base_control);

                msg_unity_control.data = is_reset;
                _pub_base_restart->publish(msg_unity_control);

                msg_unity_control.data = is_emergency;
                _pub_base_emergency->publish(msg_unity_control);

                msg_unity_control.data = is_move_autonomous;
                _pub_move_auto->publish(msg_unity_control);

                msg_unity_control.data = is_arm_autonomous;
                _pub_base_arm->publish(msg_unity_control);
            }
            if(msg->data == "s")
            {
                _pub_canusb->publish(*msg_restart);
                _pub_canusb->publish(*msg_emergency);

            }
            if(flag_restart == true)
            {
                msg_base_control.is_restart = false;
                _pub_base_control->publish(msg_base_control);
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
        
        //コントローラから回収情報をsubscribe
        void SmartphoneGamepad::callback_state_num_R2(const std_msgs::msg::String::SharedPtr msg)
        {
            const unsigned char data[2] = {msg->data[0], msg->data[1]};
            command.state_num_R2(data, r2_pc,udp_port_state);
        }
        //コントローラから射出情報をsubsclib
        void SmartphoneGamepad::callback_main(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            ///mainから射出可能司令のsub。上物の収束状況。
            RCLCPP_INFO(this->get_logger(), "can_rx_202");
            is_arm_convergence = static_cast<bool>(msg->candata[0]);
        }

        //splineからの情報をsubsclib
        void SmartphoneGamepad::callback_spline(const std_msgs::msg::Bool::SharedPtr msg)
        {
            is_spline_convergence = msg->data;
        }
        //arm_param_calculatorの情報をsubscribe
        //この関数が2つあるのは射出機構が2つあるため

        void SmartphoneGamepad::callback_arm_calculator(const std_msgs::msg::Bool::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(), "false");
             //arm_calculatorから上モノ指令値計算収束のsub。上物の指令値の収束情報。
            is_arm_calculator_convergence = msg->data;
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

        //ジョイスティックの値
        void SmartphoneGamepad::_recv_joy_main(const unsigned char data[16])
        {
            float values[4];
            //memcpy関数で指定したバイト数分のメモリをコピー
            memcpy(values, data, sizeof(float)*4);
            //ジョイスティックのベクトル
            //スティックに入力されたXとYをcanidとcandlcのパラメータを格納
            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = can_linear_id;
            msg_linear->candlc = 8;
            //ジョイスティックの回転
            //回転の値をcanidとcandlcのパラメータを格納
            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = can_angular_id;
            msg_angular->candlc = 4;
            //twistの型を変数に格納
            auto msg_gazebo = std::make_shared<geometry_msgs::msg::Twist>();

            bool flag_move_autonomous = false;
            
            uint8_t _candata_joy[8];
            //手動モードのとき
            if(is_move_autonomous == false)
            {
                //低速モードのとき
                if(is_slow_speed == true)
                {
                    //低速モード時の速度、加速度、回転をslow_velPlanner_linearに格納
                    slow_velPlanner_linear_x.vel(static_cast<double>(values[1]));//unityとロボットにおける。xとyが違うので逆にしている。
                    slow_velPlanner_linear_y.vel(static_cast<double>(-values[0]));
                    velPlanner_angular_z.vel(static_cast<double>(-values[2]));
                    //cycle関数で演算処理をかけている
                    slow_velPlanner_linear_x.cycle();
                    slow_velPlanner_linear_y.cycle();
                    velPlanner_angular_z.cycle();
                    //floatからバイト(メモリ)に変換
                    float_to_bytes(_candata_joy, static_cast<float>(slow_velPlanner_linear_x.vel()) * slow_manual_linear_max_vel);
                    float_to_bytes(_candata_joy+4, static_cast<float>(slow_velPlanner_linear_y.vel()) * slow_manual_linear_max_vel);
                    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_angular_z.vel()) * manual_angular_max_vel);
                    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];
                    
                    //msg_gazeboに速度計画機の値を格納
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

                    float_to_bytes(_candata_joy, static_cast<float>(high_velPlanner_linear_x.vel()) * high_manual_linear_max_vel);
                    float_to_bytes(_candata_joy+4, static_cast<float>(high_velPlanner_linear_y.vel()) * high_manual_linear_max_vel);
                    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_angular_z.vel()) * manual_angular_max_vel);
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