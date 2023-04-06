#include "controller_interface/controller_interface_node.hpp"

using namespace utils;

namespace controller_interface
{
    #define BUFSIZE 1024
    using std::string;
    
    SmartphoneGamepad::SmartphoneGamepad(const rclcpp::NodeOptions &options) : SmartphoneGamepad("", options) {}
    SmartphoneGamepad::SmartphoneGamepad(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("controller_interface_node", name_space, options),
        limit_linear(DBL_MAX,
        get_parameter("linear_max_vel").as_double(),
        get_parameter("linear_max_acc").as_double(),
        get_parameter("linear_max_dec").as_double() ),
        limit_angular(DBL_MAX,
        dtor(get_parameter("angular_max_vel").as_double()),
        dtor(get_parameter("angular_max_acc").as_double()),
        dtor(get_parameter("angular_max_dec").as_double()) ),
        limit_injection(DBL_MAX,
        dtor(get_parameter("injection_max_vel").as_double()),
        dtor(get_parameter("injection_max_acc").as_double()),
        dtor(get_parameter("injection_max_dec").as_double()) ),
        
        defalt_pitch(static_cast<float>(get_parameter("defalt_pitch").as_double())),
        manual_linear_max_vel(static_cast<float>(get_parameter("linear_max_vel").as_double())),
        manual_angular_max_vel(dtor(static_cast<float>(get_parameter("angular_max_vel").as_double()))),
        manual_injection_max_vel(dtor(static_cast<float>(get_parameter("injection_max_vel").as_double()))),
        defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
        defalt_wheel_autonomous_flag(get_parameter("defalt_wheel_autonomous_flag").as_bool()),
        defalt_injection_autonomous_flag(get_parameter("defalt_injection_autonomous_flag").as_bool()),
        defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool()),
        defalt_injection_m0_flag(get_parameter("defalt_injection_m0_flag").as_bool()),
        udp_port_ER_main(get_parameter("udp_port_ER_main").as_int()),
        udp_port_ER_sub(get_parameter("udp_port_ER_sub").as_int()),
        udp_port_RR(get_parameter("udp_port_RR").as_int())
        {  
            const auto heartbeat_ms = this->get_parameter("heartbeat_ms").as_int();

            //controllerからsub
            _sub_pad_er_main = this->create_subscription<controller_interface_msg::msg::SubPad>(
                "sub_pad_er_main",
                _qos,
                std::bind(&SmartphoneGamepad::callback_pad_er_main, this, std::placeholders::_1)
            );

            _sub_pad_er_sub = this->create_subscription<controller_interface_msg::msg::SubPad>(
                "sub_pad_er_sub",
                _qos,
                std::bind(&SmartphoneGamepad::callback_pad_er_sub, this, std::placeholders::_1)
            );

            _sub_pad_rr = this->create_subscription<controller_interface_msg::msg::SubPad>(
                "sub_pad_rr",
                _qos,
                std::bind(&SmartphoneGamepad::callback_pad_rr, this, std::placeholders::_1)
            );

            //mainからsub
            _sub_main_injection_possible = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_201",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main, this, std::placeholders::_1)
            );

            _sub_main_injection_complete = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_202",
                _qos,
                std::bind(&SmartphoneGamepad::callback_main, this, std::placeholders::_1)
            );

            //spline_pidからsub
            _sub_spline = this->create_subscription<std_msgs::msg::Bool>(
                "is_wheel_tracking",
                _qos,
                std::bind(&SmartphoneGamepad::callback_spline, this, std::placeholders::_1)
            );

            //injection_param_calculatorからsub
            _sub_injection_calculator_er_left = this->create_subscription<std_msgs::msg::Bool>(
                "is_calculator_convergenced_0",
                _qos,
                std::bind(&SmartphoneGamepad::callback_injection_calculator_er_left, this, std::placeholders::_1)
            );

            _sub_injection_calculator_er_right = this->create_subscription<std_msgs::msg::Bool>(
                "is_calculator_convergenced_1",
                _qos,
                std::bind(&SmartphoneGamepad::callback_injection_calculator_er_right, this, std::placeholders::_1)
            );

            _sub_injection_calculator_rr = this->create_subscription<std_msgs::msg::Bool>(
                "is_calculator_convergenced_right",
                _qos,
                std::bind(&SmartphoneGamepad::callback_injection_calculator_rr, this, std::placeholders::_1)
            );

            //common_processからsub
            _sub_common_base_control = this->create_subscription<controller_interface_msg::msg::BaseControl>(
                "pub_base_control",
                _qos,
                std::bind(&SmartphoneGamepad::callback_common_base_control, this, std::placeholders::_1)
            );

            //canusbへpub
            _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

            //controllerへpub
            _pub_convergence = this->create_publisher<controller_interface_msg::msg::Convergence>("pub_convergence" , _qos);

            _pub_scrn = this->create_publisher<controller_interface_msg::msg::SubScrn>("sub_scrn" , _qos);

            //各nodeへリスタートと手自動の切り替えをpub。
            _pub_common_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("sub_base_control",_qos);

            //デフォルト値をpub.。各種、boolに初期値を代入。
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = defalt_restart_flag;
            msg_base_control->is_emergency = defalt_emergency_flag;
            msg_base_control->is_wheel_autonomous = defalt_wheel_autonomous_flag;
            msg_base_control->is_injection_autonomous = defalt_injection_autonomous_flag;
            msg_base_control->is_injection_m0 = defalt_injection_m0_flag;
            this->is_reset = defalt_restart_flag;
            this->is_emergency = defalt_emergency_flag;
            this->is_wheel_autonomous = defalt_wheel_autonomous_flag;
            this->is_injection_autonomous = defalt_injection_autonomous_flag;
            this->is_injection_m0 = defalt_injection_m0_flag;
            _pub_common_base_control->publish(*msg_base_control);

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;
            _pub_canusb->publish(*msg_emergency);

            //ハートビート
            _pub_timer = this->create_wall_timer(
                std::chrono::milliseconds(heartbeat_ms),
                [this] { 
                    auto msg_heartbeat = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_heartbeat->canid = 0x001;
                    msg_heartbeat->candlc = 0;
                    _pub_canusb->publish(*msg_heartbeat);
                }
            );

            //計画機
            velPlanner_linear_x.limit(limit_linear);
            velPlanner_linear_y.limit(limit_linear);
            velPlanner_angular_z.limit(limit_angular);
            velPlanner_injection_v.limit(limit_injection);

            //UDP
            sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;
            servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
            servaddr.sin_port = htons(udp_port_ER_main);
            bind(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr));
            udp_thread_ = std::thread(&SmartphoneGamepad::callback_udp_er_main, this, sockfd);

            sockfd2 = socket(AF_INET, SOCK_DGRAM, 0);
            memset(&servaddr2, 0, sizeof(servaddr2));
            servaddr2.sin_family = AF_INET;
            servaddr2.sin_addr.s_addr = htonl(INADDR_ANY);
            servaddr2.sin_port = htons(udp_port_ER_sub);
            bind(sockfd2, (struct sockaddr *) &servaddr2, sizeof(servaddr2));
            udp_thread_2 = std::thread(&SmartphoneGamepad::callback_udp_er_sub, this, sockfd2);

            sockfd3 = socket(AF_INET, SOCK_DGRAM, 0);
            memset(&servaddr3, 0, sizeof(servaddr3));
            servaddr3.sin_family = AF_INET;
            servaddr3.sin_addr.s_addr = htonl(INADDR_ANY);
            servaddr3.sin_port = htons(udp_port_RR);
            bind(sockfd3, (struct sockaddr *) &servaddr3, sizeof(servaddr3));
            udp_thread_3 = std::thread(&SmartphoneGamepad::callback_udp_rr, this, sockfd3);
        }

        void SmartphoneGamepad::callback_pad_er_main(const controller_interface_msg::msg::SubPad::SharedPtr msg)
        {
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = 0x002;
            msg_restart->candlc = 0;

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = 0x300;
            msg_btn->candlc = 8;

            auto msg_sub_scrn = std::make_shared<controller_interface_msg::msg::SubScrn>();

            uint8_t _candata_btn[8];

            bool robotcontrol_flag = false;//base_control(手自動、緊急、リスタート)が押されたらpubする
            bool flag_restart = false;//resertがtureをpubした後にfalseをpubする

            //r3は足回りの手自動の切り替え。is_wheel_autonomousを使って、トグルになるようにしてる。ERの上物からもらう必要はない。
            //ERの上物の場合は、上物の切り替えに当てている。
            
            if(msg->r3)
            {
                robotcontrol_flag = true;
                if(is_wheel_autonomous == false) is_wheel_autonomous = true;
                else is_wheel_autonomous = false;
            }

            //gは緊急。is_emergencyを使って、トグルになるようにしてる。
            if(msg->g)
            {
                robotcontrol_flag = true;
                if(is_emergency == false) is_emergency = true;
                else is_emergency = false;
            }

            //sはリスタート。緊急と手自動のboolをfalseにしてリセットしている。
            if(msg->s)
            {
                robotcontrol_flag = true;
                flag_restart = true;
                is_wheel_autonomous = defalt_wheel_autonomous_flag;
                is_injection_autonomous = defalt_injection_autonomous_flag;
                is_emergency = defalt_emergency_flag;
                is_injection_m0 = defalt_injection_m0_flag;
            }

            is_reset = msg->s;

            //basecontrolへの代入
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = is_reset;
            msg_base_control->is_emergency = is_emergency;
            msg_base_control->is_wheel_autonomous = is_wheel_autonomous;
            msg_base_control->is_injection_autonomous = is_injection_autonomous;
            msg_base_control->is_injection_m0 = is_injection_m0;

            
            //mainへ緊急を送る代入
            _candata_btn[0] = is_emergency;
            for(int i=0; i<msg_emergency->candlc; i++) msg_emergency->candata[i] = _candata_btn[i];

            //mainへボタン情報を送る代入
            _candata_btn[0] = msg->a;
            _candata_btn[1] = msg->b;
            _candata_btn[2] = msg->y;
            _candata_btn[3] = msg->x;
            _candata_btn[4] = msg->right;
            _candata_btn[5] = msg->down;
            _candata_btn[6] = msg->left;
            _candata_btn[7] = msg->up;
            for(int i=0; i<msg_btn->candlc; i++) msg_btn->candata[i] = _candata_btn[i];

            if(msg->a || msg->b || msg->y || msg->x || msg->right || msg->down || msg->left || msg->up) 
            {
                _pub_canusb->publish(*msg_btn); 
                //RCLCPP_INFO(this->get_logger(), "a:%db:%dy:%dx:%dright:%ddown:%dleft:%dup:%d", msg->a, msg->b, msg->y, msg->x, msg->right, msg->down, msg->left, msg->up);
            }
            if(msg->g)_pub_canusb->publish(*msg_emergency);
            if(robotcontrol_flag)_pub_common_base_control->publish(*msg_base_control);
            if(msg->s)
            {
                _pub_canusb->publish(*msg_restart);
                _pub_scrn->publish(*msg_sub_scrn);
            }
            if(flag_restart)
            {
                msg_base_control->is_restart = false;
                _pub_common_base_control->publish(*msg_base_control);
            }
        }

        void SmartphoneGamepad::callback_pad_er_sub(const controller_interface_msg::msg::SubPad::SharedPtr msg)
        {
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = 0x002;
            msg_restart->candlc = 0;

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            auto msg_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_injection->canid = 0x200;
            msg_injection->candlc = 2;

            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = 0x300;
            msg_btn->candlc = 8;

            auto msg_sub_scrn = std::make_shared<controller_interface_msg::msg::SubScrn>();

            uint8_t _candata_btn[8];

            bool robotcontrol_flag = false;//base_control(手自動、緊急、リスタート)が押されたらpubする
            bool flag_restart = false;//resertがtureをpubした後にfalseをpubする
            bool flag_injection0 = false;//左の発射機構の最終射出許可
            bool flag_injection1 = false;//右の発射機構の最終射出許可

            //r3は足回りの手自動の切り替え。is_wheel_autonomousを使って、トグルになるようにしてる。ERの上物からもらう必要はない。
            //ERの上物の場合は、上物の切り替えに当てている。
            
            if(msg->r3)
            {
                robotcontrol_flag = true;
                if(is_injection_m0 == false) is_injection_m0 = true;
                else is_injection_m0 = false;
            }

            //l3は上物の手自動の切り替え。is_injection_autonomousを使って、トグルになるようにしてる。ERの足回りからもらう必要はない
            if(msg->l3)
            {
                robotcontrol_flag = true;
                if(is_injection_autonomous == false) is_injection_autonomous = true;
                else is_injection_autonomous = false;
            }

            //gは緊急。is_emergencyを使って、トグルになるようにしてる。
            if(msg->g)
            {
                robotcontrol_flag = true;
                if(is_emergency == false) is_emergency = true;
                else is_emergency = false;
            }

            //sはリスタート。緊急と手自動のboolをfalseにしてリセットしている。
            if(msg->s)
            {
                msg_sub_scrn->a = false;
                msg_sub_scrn->b = false;
                msg_sub_scrn->c = false;
                msg_sub_scrn->d = false;
                msg_sub_scrn->e = false;
                msg_sub_scrn->f = false;
                msg_sub_scrn->g = false;
                msg_sub_scrn->h = false;
                msg_sub_scrn->i = false;
                msg_sub_scrn->j = false;
                msg_sub_scrn->k = false;
            }

            //l2が左、r2が右の発射機構のトリガー。
            //それぞれ、発射されたら収束がfalseにするようにしている。
            if(msg->l2)
            {
                if(is_spline_convergence && is_injection0_convergence && is_injection_calculator0_convergence)
                {
                    flag_injection0 = true;
                    is_injection0_convergence = false;
                    is_injection_calculator0_convergence = false;
                }
            }
            
            if(msg->r2)
            {
                if(is_spline_convergence && is_injection1_convergence && is_injection_calculator1_convergence)
                {
                    flag_injection1 = true;
                    is_injection1_convergence = false;
                    is_injection_calculator1_convergence = false;
                }
            }

            is_reset = msg->s;

            //basecontrolへの代入
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = is_reset;
            msg_base_control->is_emergency = is_emergency;
            msg_base_control->is_wheel_autonomous = is_wheel_autonomous;
            msg_base_control->is_injection_autonomous = is_injection_autonomous;
            msg_base_control->is_injection_m0 = is_injection_m0;
    
            //mainへ緊急を送る代入
            _candata_btn[0] = is_emergency;
            for(int i=0; i<msg_emergency->candlc; i++) msg_emergency->candata[i] = _candata_btn[i];

            //mainへ射出司令を送る代入
            _candata_btn[0] = flag_injection0;
            _candata_btn[1] = flag_injection1;
            for(int i=0; i<msg_injection->candlc; i++) msg_injection->candata[i] = _candata_btn[i];

            //mainへボタン情報を送る代入
            _candata_btn[0] = msg->a;
            _candata_btn[1] = msg->b;
            _candata_btn[2] = msg->y;
            _candata_btn[3] = msg->x;
            _candata_btn[4] = msg->right;
            _candata_btn[5] = msg->down;
            _candata_btn[6] = msg->left;
            _candata_btn[7] = msg->up;
            for(int i=0; i<msg_btn->candlc; i++) msg_btn->candata[i] = _candata_btn[i];

            if(msg->a || msg->b || msg->y || msg->x || msg->right || msg->down || msg->left || msg->up) 
            {
                _pub_canusb->publish(*msg_btn); 
            }
            if(msg->g)_pub_canusb->publish(*msg_emergency);
            if(flag_injection0 || flag_injection1)_pub_canusb->publish(*msg_injection);
            if(robotcontrol_flag)_pub_common_base_control->publish(*msg_base_control);
            if(msg->s)
            { 
                _pub_scrn->publish(*msg_sub_scrn);
            }
            if(flag_restart)
            {
                msg_base_control->is_restart = false;
                _pub_common_base_control->publish(*msg_base_control);
            }
        }

        void SmartphoneGamepad::callback_pad_rr(const controller_interface_msg::msg::SubPad::SharedPtr msg)
        {
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = 0x002;
            msg_restart->candlc = 0;

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            auto msg_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_injection->canid = 0x200;
            msg_injection->candlc = 2;

            auto msg_btn = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_btn->canid = 0x300;
            msg_btn->candlc = 8;

            auto msg_sub_scrn = std::make_shared<controller_interface_msg::msg::SubScrn>();

            uint8_t _candata_btn[8];

            bool robotcontrol_flag = false;//base_control(手自動、緊急、リスタート)が押されたらpubする
            bool flag_restart = false;//resertがtureをpubした後にfalseをpubする
            bool flag_injection0 = false;//左の発射機構の最終射出許可
            bool flag_injection1 = false;//右の発射機構の最終射出許可

            //r3は足回りの手自動の切り替え。is_wheel_autonomousを使って、トグルになるようにしてる。ERの上物からもらう必要はない。
            //ERの上物の場合は、上物の切り替えに当てている。
            
            if(msg->r3)
            {
                robotcontrol_flag = true;
                if(is_wheel_autonomous == false) is_wheel_autonomous = true;
                else is_wheel_autonomous = false;
            }

            //gは緊急。is_emergencyを使って、トグルになるようにしてる。
            if(msg->g)
            {
                robotcontrol_flag = true;
                if(is_emergency == false) is_emergency = true;
                else is_emergency = false;
            }

            //sはリスタート。緊急と手自動のboolをfalseにしてリセットしている。
            
            if(msg->s)
            {
                msg_sub_scrn->a = false;
                msg_sub_scrn->b = false;
                msg_sub_scrn->c = false;
                msg_sub_scrn->d = false;
                msg_sub_scrn->e = false;
                msg_sub_scrn->f = false;
                msg_sub_scrn->g = false;
                msg_sub_scrn->h = false;
                msg_sub_scrn->i = false;
                msg_sub_scrn->j = false;
                msg_sub_scrn->k = false;

                robotcontrol_flag = true;
                flag_restart = true;
                is_wheel_autonomous = defalt_wheel_autonomous_flag;
                is_injection_autonomous = defalt_injection_autonomous_flag;
                is_emergency = defalt_emergency_flag;
                is_injection_m0 = defalt_injection_m0_flag;
            }

            //l2が左、r2が右の発射機構のトリガー。
            //それぞれ、発射されたら収束がfalseにするようにしている。
            if(msg->l2)
            {
                if(is_spline_convergence && is_injection0_convergence && is_injection_calculator0_convergence)
                {
                    flag_injection0 = true;
                    is_injection0_convergence = false;
                    is_injection_calculator0_convergence = false;
                }
            }

            is_reset = msg->s;

            //basecontrolへの代入
            auto msg_base_control = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_control->is_restart = is_reset;
            msg_base_control->is_emergency = is_emergency;
            msg_base_control->is_wheel_autonomous = is_wheel_autonomous;
            msg_base_control->is_injection_autonomous = is_injection_autonomous;
            msg_base_control->is_injection_m0 = is_injection_m0;

            //mainへ緊急を送る代入
            _candata_btn[0] = is_emergency;
            for(int i=0; i<msg_emergency->candlc; i++) msg_emergency->candata[i] = _candata_btn[i];

            //mainへ射出司令を送る代入
            _candata_btn[0] = flag_injection0;
            _candata_btn[1] = flag_injection1;
            for(int i=0; i<msg_injection->candlc; i++) msg_injection->candata[i] = _candata_btn[i];

            //mainへボタン情報を送る代入
            _candata_btn[0] = msg->a;
            _candata_btn[1] = msg->b;
            _candata_btn[2] = msg->y;
            _candata_btn[3] = msg->x;
            _candata_btn[4] = msg->right;
            _candata_btn[5] = msg->down;
            _candata_btn[6] = msg->left;
            _candata_btn[7] = msg->up;
            for(int i=0; i<msg_btn->candlc; i++) msg_btn->candata[i] = _candata_btn[i];

            if(msg->a || msg->b || msg->y || msg->x || msg->right || msg->down || msg->left || msg->up) 
            {
                _pub_canusb->publish(*msg_btn); 
                RCLCPP_INFO(this->get_logger(), "a:%db:%dy:%dx:%dright:%ddown:%dleft:%dup:%d", msg->a, msg->b, msg->y, msg->x, msg->right, msg->down, msg->left, msg->up);
            }
            if(msg->g)_pub_canusb->publish(*msg_emergency);
            if(flag_injection0 || flag_injection1)_pub_canusb->publish(*msg_injection);
            if(robotcontrol_flag)_pub_common_base_control->publish(*msg_base_control);
            if(msg->s)
            {
                _pub_canusb->publish(*msg_restart);
                _pub_scrn->publish(*msg_sub_scrn);
            }
            if(flag_restart)
            {
                msg_base_control->is_restart = false;
                _pub_common_base_control->publish(*msg_base_control);
            }
        }

        void SmartphoneGamepad::callback_udp_er_main(int sockfd)
        {
            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = 0x100;
            msg_linear->candlc = 8;

            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = 0x101;
            msg_angular->candlc = 4;

            uint8_t _candata_joy[8];

            bool flag_wheel_autonomous = false;

            float analog_l_x = 0.0f;
            float analog_l_y = 0.0f;
            float analog_r_x = 0.0f;
            float analog_r_y = 0.0f;

            while(rclcpp::ok())
            {
                clilen = sizeof(cliaddr);
                    
                // bufferに受信したデータが格納されている
                n = recvfrom(sockfd, buffer, BUFSIZE, 0, (struct sockaddr *) &cliaddr, &clilen);
                    
                if (n < 0)
                {
                    perror("recvfrom");
                    exit(1);
                }

                std::memcpy(&analog_l_x, &buffer[0], sizeof(analog_l_x));
                std::memcpy(&analog_l_y, &buffer[4], sizeof(analog_l_y));
                std::memcpy(&analog_r_x, &buffer[8], sizeof(analog_r_x));
                std::memcpy(&analog_r_y, &buffer[12], sizeof(analog_r_y));

                if(is_wheel_autonomous == false)
                {
                    velPlanner_linear_x.vel(static_cast<double>(analog_l_y));//unityとロボットにおける。xとyが違うので逆にしている。
                    velPlanner_linear_y.vel(static_cast<double>(-analog_l_x));
                    velPlanner_angular_z.vel(static_cast<double>(-analog_r_x));

                    velPlanner_linear_x.cycle();
                    velPlanner_linear_y.cycle();
                    velPlanner_angular_z.cycle();

                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_linear_x.vel()) * manual_linear_max_vel);
                    float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_linear_y.vel()) * manual_linear_max_vel);
                    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_angular_z.vel()) * manual_angular_max_vel);
                    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_linear);
                    _pub_canusb->publish(*msg_angular);

                    flag_wheel_autonomous = true;
                }
                else 
                {
                    //手動から自動になったときに、一回だけ速度指令値に0を代入してpubする。
                    if(flag_wheel_autonomous == true)
                    {
                        float_to_bytes(_candata_joy, 0);
                        for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                        _pub_canusb->publish(*msg_linear);
                        _pub_canusb->publish(*msg_angular);

                        flag_wheel_autonomous = false;
                    }
                }
            }
        }

        void SmartphoneGamepad::callback_udp_er_sub(int sockfd)
        {
            auto msg_l_elevation_velocity = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_l_elevation_velocity->canid = 0x210;
            msg_l_elevation_velocity->candlc = 8;

            auto msg_l_yaw = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_l_yaw->canid = 0x211;
            msg_l_yaw->candlc = 4;

            auto msg_r_elevation_velocity = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_r_elevation_velocity->canid = 0x212;
            msg_r_elevation_velocity->candlc = 8;

            auto msg_r_yaw = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_r_yaw->canid = 0x213;
            msg_r_yaw->candlc = 4;

            uint8_t _candata_joy[8];

            bool flag_injection_autonomous = false;

            float analog_l_x = 0.0f;
            float analog_l_y = 0.0f;
            float analog_r_x = 0.0f;
            float analog_r_y = 0.0f;

            float pitch = defalt_pitch;

            while(rclcpp::ok())
            {
                clilen = sizeof(cliaddr);
                    
                // bufferに受信したデータが格納されている
                n = recvfrom(sockfd, buffer, BUFSIZE, 0, (struct sockaddr *) &cliaddr, &clilen);
                    
                if (n < 0)
                {
                    perror("recvfrom");
                    exit(1);
                }

                std::memcpy(&analog_l_x, &buffer[0], sizeof(analog_l_x));
                std::memcpy(&analog_l_y, &buffer[4], sizeof(analog_l_y));
                std::memcpy(&analog_r_x, &buffer[8], sizeof(analog_r_x));
                std::memcpy(&analog_r_y, &buffer[12], sizeof(analog_r_y));

                if(is_injection_autonomous == false)
                {
                    velPlanner_injection_v.vel(static_cast<double>(-analog_l_x));

                    velPlanner_injection_v.cycle();

                    if(is_injection_m0)
                    {
                        float_to_bytes(_candata_joy, pitch);
                        float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_injection_v.vel()) * manual_injection_max_vel);
                        for(int i=0; i<msg_l_elevation_velocity->candlc; i++) msg_l_elevation_velocity->candata[i] = _candata_joy[i];

                        float_to_bytes(_candata_joy, static_cast<float>(atan2(-analog_r_x, analog_r_y)));
                        for(int i=0; i<msg_l_yaw->candlc; i++) msg_l_yaw->candata[i] = _candata_joy[i];

                        _pub_canusb->publish(*msg_l_elevation_velocity);
                        _pub_canusb->publish(*msg_l_yaw);
                    }
                    else
                    {
                        float_to_bytes(_candata_joy, pitch);
                        float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_injection_v.vel()) * manual_injection_max_vel);
                        for(int i=0; i<msg_r_elevation_velocity->candlc; i++) msg_r_elevation_velocity->candata[i] = _candata_joy[i];

                        float_to_bytes(_candata_joy, static_cast<float>(atan2(-analog_r_x, analog_r_y)));
                        for(int i=0; i<msg_r_yaw->candlc; i++) msg_r_yaw->candata[i] = _candata_joy[i];

                        _pub_canusb->publish(*msg_r_elevation_velocity);
                        _pub_canusb->publish(*msg_r_yaw);
                    }

                    flag_injection_autonomous = true;
                }
                else 
                {
                    //手動から自動になったときに、一回だけ速度指令値に0を代入してpubする。
                    if(flag_injection_autonomous == true)
                    {
                        float_to_bytes(_candata_joy, 0);
                        for(int i=0; i<msg_l_elevation_velocity->candlc; i++) msg_l_elevation_velocity->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_l_yaw->candlc; i++) msg_l_yaw->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_r_elevation_velocity->candlc; i++) msg_r_elevation_velocity->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_r_yaw->candlc; i++) msg_r_yaw->candata[i] = _candata_joy[i];

                        _pub_canusb->publish(*msg_l_elevation_velocity);
                        _pub_canusb->publish(*msg_l_yaw);
                        _pub_canusb->publish(*msg_r_elevation_velocity);
                        _pub_canusb->publish(*msg_r_yaw);

                        flag_injection_autonomous = false;
                    }
                }
            }
        }

        void SmartphoneGamepad::callback_udp_rr(int sockfd)
        {
            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = 0x100;
            msg_linear->candlc = 8;

            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = 0x101;
            msg_angular->candlc = 4;

            auto msg_l_elevation_velocity = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_l_elevation_velocity->canid = 0x210;
            msg_l_elevation_velocity->candlc = 8;

            auto msg_l_yaw = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_l_yaw->canid = 0x211;
            msg_l_yaw->candlc = 4;

            uint8_t _candata_joy[8];

            bool flag_wheel_autonomous = false;
            bool flag_injection_autonomous = false;

            float analog_l_x = 0.0f;
            float analog_l_y = 0.0f;
            float analog_r_x = 0.0f;
            float analog_r_y = 0.0f;

            float pitch = defalt_pitch;

            while(rclcpp::ok())
            {
                clilen = sizeof(cliaddr);
                    
                // bufferに受信したデータが格納されている
                n = recvfrom(sockfd, buffer, BUFSIZE, 0, (struct sockaddr *) &cliaddr, &clilen);
                    
                if (n < 0)
                {
                    perror("recvfrom");
                    exit(1);
                }

                std::memcpy(&analog_l_x, &buffer[0], sizeof(analog_l_x));
                std::memcpy(&analog_l_y, &buffer[4], sizeof(analog_l_y));
                std::memcpy(&analog_r_x, &buffer[8], sizeof(analog_r_x));
                std::memcpy(&analog_r_y, &buffer[12], sizeof(analog_r_y));

                if(is_wheel_autonomous == false && is_injection_autonomous == true)
                {
                    velPlanner_linear_x.vel(static_cast<double>(analog_l_y));//unityとロボットにおける。xとyが違うので逆にしている。
                    velPlanner_linear_y.vel(static_cast<double>(-analog_l_x));
                    velPlanner_angular_z.vel(static_cast<double>(-analog_r_x));

                    velPlanner_linear_x.cycle();
                    velPlanner_linear_y.cycle();
                    velPlanner_angular_z.cycle();

                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_linear_x.vel()) * manual_linear_max_vel);
                    float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_linear_y.vel()) * manual_linear_max_vel);
                    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_angular_z.vel()) * manual_angular_max_vel);
                    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_linear);
                    _pub_canusb->publish(*msg_angular);

                    flag_wheel_autonomous = true;
                }
                else if(is_wheel_autonomous == true && is_injection_autonomous == false)
                {
                    velPlanner_injection_v.vel(static_cast<double>(-analog_l_x));
                    velPlanner_angular_z.vel(static_cast<double>(-analog_r_x));

                    velPlanner_injection_v.cycle();
                    velPlanner_angular_z.cycle();

                    float_to_bytes(_candata_joy, pitch);
                    float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_injection_v.vel()) * manual_injection_max_vel);
                    for(int i=0; i<msg_l_elevation_velocity->candlc; i++) msg_l_elevation_velocity->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_angular_z.vel()) * manual_angular_max_vel);
                    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_l_elevation_velocity);
                    _pub_canusb->publish(*msg_angular);

                    flag_injection_autonomous = true;
                }
                else 
                {
                    //手動から自動になったときに、一回だけ速度指令値に0を代入してpubする。
                    if(flag_wheel_autonomous == true || flag_injection_autonomous == true)
                    {
                        float_to_bytes(_candata_joy, 0);
                        for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_l_elevation_velocity->candlc; i++) msg_l_elevation_velocity->candata[i] = _candata_joy[i];
                        for(int i=0; i<msg_l_yaw->candlc; i++) msg_l_yaw->candata[i] = _candata_joy[i];

                        _pub_canusb->publish(*msg_linear);
                        _pub_canusb->publish(*msg_angular);
                        _pub_canusb->publish(*msg_l_elevation_velocity);
                        _pub_canusb->publish(*msg_l_yaw);

                        flag_wheel_autonomous = false;
                        flag_injection_autonomous = false;
                    }
                }
            }
        }

        void SmartphoneGamepad::callback_common_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg)
        {
            //CommonProsesからのBaseContolをsubしてコントローラとの同期をする
            is_reset = msg->is_restart;
            is_emergency = msg->is_emergency;
            is_wheel_autonomous = msg->is_wheel_autonomous; 
            is_injection_autonomous = msg->is_injection_autonomous;
            is_injection_m0 = msg->is_injection_m0;
        }

        void SmartphoneGamepad::callback_main(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg)
        {
            //mainから射出可能司令のsub。それぞれをconvergenceの適当なところに入れてpub。上物の収束状況。
            auto msg_injectioncommnd = std::make_shared<controller_interface_msg::msg::Convergence>();
            uint8_t _candata[2];
            for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
            msg_injectioncommnd->injection0 = static_cast<bool>(_candata[0]);
            msg_injectioncommnd->injection1 = static_cast<bool>(_candata[1]);
            _pub_convergence->publish(*msg_injectioncommnd);
        }

        void SmartphoneGamepad::callback_spline(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //spline_pidから足回り収束のsub。onvergenceの適当なところに入れてpub。足回りの収束状況。
            auto msg_spline_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            is_spline_convergence = msg->data;
            msg_spline_convergence->spline_convergence = is_spline_convergence;
            _pub_convergence->publish(*msg_spline_convergence);
        }

        void SmartphoneGamepad::callback_injection_calculator_er_left(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //injection_calculatorから上モノ指令値計算収束のsub。onvergenceの適当なところに入れてpub。上物の指令値の収束情報。
            auto msg_injection_calculator0_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            is_injection_calculator0_convergence = msg->data;
            msg_injection_calculator0_convergence->injection_calculator0 = is_injection_calculator0_convergence;
            _pub_convergence->publish(*msg_injection_calculator0_convergence);
        }

        void SmartphoneGamepad::callback_injection_calculator_er_right(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //injection_calculatorから上モノ指令値計算収束のsub。onvergenceの適当なところに入れてpub。上物の指令値の収束情報。7
            auto msg_injection_calculator1_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            is_injection_calculator1_convergence = msg->data;
            msg_injection_calculator1_convergence->injection_calculator1 = is_injection_calculator1_convergence;
            _pub_convergence->publish(*msg_injection_calculator1_convergence);
        }

         void SmartphoneGamepad::callback_injection_calculator_rr(const std_msgs::msg::Bool::SharedPtr msg)
        {
            //injection_calculatorから上モノ指令値計算収束のsub。onvergenceの適当なところに入れてpub。上物の指令値の収束情報。
            // auto msg_injection_calculator1_convergence = std::make_shared<controller_interface_msg::msg::Convergence>();
            // is_injection_calculator1_convergence = msg->data;
            // msg_injection_calculator1_convergence->injection_calculator1 = is_injection_calculator1_convergence;
            // _pub_convergence->publish(*msg_injection_calculator1_convergence);
        }
    
    CommonProces::CommonProces(const rclcpp::NodeOptions &options) : CommonProces("", options) {}
    CommonProces::CommonProces(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("controller_common_proces_node", name_space, options)
        {
            //コントローラからSubScrnをsub
            _sub_tcp1_scrn = this->create_subscription<controller_interface_msg::msg::SubScrn>(
                "sub_scrn",
                _qos,
                std::bind(&CommonProces::callback_scrn, this, std::placeholders::_1)
            );

            //controller_intefaceからBaseConstrolをsub
            _sub_tcp1_base_control = this->create_subscription<controller_interface_msg::msg::BaseControl>(
                "sub_base_control",
                _qos,
                std::bind(&CommonProces::callback_base_contol_RR, this, std::placeholders::_1)
            );

            //コントローラにSubScrnをpubする
            _pub_tcp1_scrn = this->create_publisher<controller_interface_msg::msg::SubScrn>("pub_scrn",_qos);

            //コントローラにBaseControlをpubする
            _pub_tcp1_base_control = this->create_publisher<controller_interface_msg::msg::BaseControl>("pub_base_control",_qos);
        }

        void CommonProces::callback_scrn(const controller_interface_msg::msg::SubScrn::SharedPtr msg)
        {
            //各コントローラからSubScrnをsub、それを統合してコントローラにSubScrnをpubしている。
            sub_scrn[0] = msg->a;
            sub_scrn[1] = msg->b;
            sub_scrn[2] = msg->c;
            sub_scrn[3] = msg->d;
            sub_scrn[4] = msg->e;
            sub_scrn[5] = msg->f;
            sub_scrn[6] = msg->g;
            sub_scrn[7] = msg->h;
            sub_scrn[8] = msg->i;
            sub_scrn[9] = msg->j;
            sub_scrn[10] = msg->k;

            auto msg_pole_btn = std::make_shared<controller_interface_msg::msg::SubScrn>();
            msg_pole_btn->a = sub_scrn[0];
            msg_pole_btn->b = sub_scrn[1];
            msg_pole_btn->c = sub_scrn[2];
            msg_pole_btn->d = sub_scrn[3];
            msg_pole_btn->e = sub_scrn[4];
            msg_pole_btn->f = sub_scrn[5];
            msg_pole_btn->g = sub_scrn[6];
            msg_pole_btn->h = sub_scrn[7];
            msg_pole_btn->i = sub_scrn[8];
            msg_pole_btn->j = sub_scrn[9];
            msg_pole_btn->k = sub_scrn[10];
            _pub_tcp1_scrn->publish(*msg_pole_btn);
            //RCLCPP_INFO(this->get_logger(), "a:%db:%dc:%dd:%de:%df:%dg:%dh:%di:%dj:%dk:%d", msg_pole_btn->a, msg_pole_btn->b, msg_pole_btn->c, msg_pole_btn->d, msg_pole_btn->e, msg_pole_btn->f, msg_pole_btn->g, msg_pole_btn->h, msg_pole_btn->i, msg_pole_btn->j, msg_pole_btn->k);
            //RCLCPP_INFO(this->get_logger(), "a:%db:%dc:%dd:%de:%df:%dg:%dh:%di:%dj:%dk:%d", msg->a, msg->b, msg->c, msg->d, msg->e, msg->f, msg->g, msg->h, msg->i, msg->j, msg->k);
            //RCLCPP_INFO(this->get_logger(), "a:%db:%dc:%dd:%de:%df:%dg:%dh:%di:%dj:%dk:%d", sub_scrn[0], sub_scrn[1], sub_scrn[2], sub_scrn[3], sub_scrn[4], sub_scrn[5], sub_scrn[6], sub_scrn[7], sub_scrn[8], sub_scrn[9], sub_scrn[10]);
        }

        void CommonProces::callback_base_contol_RR(const controller_interface_msg::msg::BaseControl::SharedPtr msg)
        {
            //そのまま素通りでBaseControlに代入
            auto msg_base_btn = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_base_btn->is_restart = msg->is_restart;
            msg_base_btn->is_emergency = msg->is_emergency;
            msg_base_btn->is_wheel_autonomous = msg->is_wheel_autonomous;
            msg_base_btn->is_injection_autonomous = msg->is_injection_autonomous;
            msg_base_btn->is_injection_m0 = msg->is_injection_m0;
            _pub_tcp1_base_control->publish(*msg_base_btn);
            //RCLCPP_INFO(this->get_logger(), "restart():%demergency():%dwheel():%dinjection():%dinjection_m0():%d", msg_base_btn->is_restart, msg_base_btn->is_emergency, msg_base_btn->is_wheel_autonomous, msg_base_btn->is_injection_autonomous, msg_base_btn->is_injection_m0);
        }
}
