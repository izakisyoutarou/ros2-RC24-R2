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

        manual_max_vel(static_cast<float>(get_parameter("linear_max_vel").as_double())),
        defalt_restart_flag(get_parameter("defalt_restart_flag").as_bool()),
        defalt_autonomous_flag(get_parameter("defalt_wheel_autonomous_flag").as_bool()),
        defalt_emergency_flag(get_parameter("defalt_emergency_flag").as_bool())
        {
            const auto heartbeat_ms = this->get_parameter("heartbeat_ms").as_int();
            sampling_time = heartbeat_ms / 1000.0;

            //controllerからsub
            _sub_pad = this->create_subscription<controller_interface_msg::msg::SubPad>(
                "sub_pad",
                _qos,
                std::bind(&SmartphoneGamepad::callback_pad, this, std::placeholders::_1)
            );

            _sub_scrn = this->create_subscription<controller_interface_msg::msg::SubScrn>(
                "sub_scrn",
                _qos,
                std::bind(&SmartphoneGamepad::callback_scrn, this, std::placeholders::_1)
            );

            //canusbへpub
            _pub_canusb = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

            //各nodeへリスタートと手自動の切り替えをpub。デフォルト値をpub
            _pub_tool = this->create_publisher<controller_interface_msg::msg::BaseControl>("base_control",_qos);

            auto msg_tool = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_tool->is_restart = defalt_restart_flag;
            msg_tool->is_autonomous = defalt_autonomous_flag;
            this->is_reset = defalt_restart_flag;
            this->is_autonomous = defalt_autonomous_flag;
            this->is_emergency = defalt_emergency_flag;
            _pub_tool->publish(*msg_tool);

            //ハートビート
            _pub_timer = this->create_wall_timer(
                std::chrono::milliseconds(heartbeat_ms),
                [this] {
                    auto msg_heartbeat = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
                    msg_heartbeat->canid = 0x001;
                    msg_heartbeat->candlc = 1;
                    _pub_canusb->publish(*msg_heartbeat);
                }
            );

            //計画機
            velPlanner_linear_x.limit(limit_linear);
            velPlanner_linear_y.limit(limit_linear);
            velPlanner_angular_z.limit(limit_angular);

            //UDP
            // struct in_addr local_addr;
            // inet_pton(AF_INET, "192.168.1.4", &local_addr);
            sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            memset(&servaddr, 0, sizeof(servaddr));
            servaddr.sin_family = AF_INET;
            //servaddr.sin_addr = local_addr;
            servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
            servaddr.sin_port = htons(5000);
            bind(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr));

            //UDPthread
            udp_thread_ = std::thread(&SmartphoneGamepad::callback_udp, this);
        }

        void SmartphoneGamepad::callback_pad(const controller_interface_msg::msg::SubPad::SharedPtr msg)
        {
            auto msg_restart = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_restart->canid = 0x002;
            msg_restart->candlc = 0;

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            uint8_t _candata_btn;

            bool robotcontrol_flag = false;

            if(msg->r3)
            {
                robotcontrol_flag = true;
                if(is_autonomous == false) is_autonomous = true;
                else is_autonomous = false;
            }

           if(msg->g)
            {
                robotcontrol_flag = true;
                if(is_emergency == false) is_emergency = true;
                else is_emergency = false;
            }

            if(msg->s)
            {
                robotcontrol_flag = true;
                is_autonomous = defalt_autonomous_flag;
                is_emergency = defalt_emergency_flag;
            }

            is_reset = msg->s;
            //RCLCPP_INFO(this->get_logger(), "flag:%d", flag);

            auto msg_tool = std::make_shared<controller_interface_msg::msg::BaseControl>();
            msg_tool->is_restart = is_reset;
            msg_tool->is_autonomous = is_autonomous;
            msg_tool->is_emergency = is_emergency;

            _candata_btn = is_reset;
            for(int i=0; i<msg_restart->candlc; i++) msg_restart->candata[i] = _candata_btn;

            _candata_btn = is_emergency;
            for(int i=0; i<msg_emergency->candlc; i++) msg_emergency->candata[i] = _candata_btn;

            if(msg->g)_pub_canusb->publish(*msg_emergency);
            if(msg->s)_pub_canusb->publish(*msg_restart);
            if(robotcontrol_flag)_pub_tool->publish(*msg_tool);
        }

        void SmartphoneGamepad::callback_scrn(const controller_interface_msg::msg::SubScrn::SharedPtr msg)
        {
            //スマホのスクリーンボタンの押されたボタンを受信するcall_back
            //RCLCPP_INFO(this->get_logger(), "flag:%d", msg->p1);
        }

        void SmartphoneGamepad::callback_udp()
        {
            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = 0x100;
            msg_linear->candlc = 8;

            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = 0x101;
            msg_angular->candlc = 4;

            uint8_t _candata_joy[8];

            bool flag_autonomous = false;

            while(rclcpp::ok())
            {
                if(is_autonomous == false)
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

                    velPlanner_linear_x.vel(upcast(analog_l_y));//unityとロボットにおける。xとyが違うので逆にしている。
                    velPlanner_linear_y.vel(upcast(analog_l_x));
                    velPlanner_angular_z.vel(upcast(analog_r_x));

                    velPlanner_linear_x.cycle();
                    velPlanner_linear_y.cycle();
                    velPlanner_angular_z.cycle();

                    //RCLCPP_INFO(this->get_logger(), "vel_x:%f", analog_l_y);

                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_linear_x.vel()) * manual_max_vel);
                    float_to_bytes(_candata_joy+4, static_cast<float>(velPlanner_linear_y.vel()) * manual_max_vel);
                    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                    float_to_bytes(_candata_joy, static_cast<float>(velPlanner_angular_z.vel()) * manual_max_vel);
                    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                    _pub_canusb->publish(*msg_linear);
                    _pub_canusb->publish(*msg_angular);

                    flag_autonomous = true;
                }
                else
                {
                    if(flag_autonomous == true)
                    {
                        float_to_bytes(_candata_joy, 0);
                        float_to_bytes(_candata_joy+4, 0);
                        for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                        float_to_bytes(_candata_joy, 0);
                        for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                        _pub_canusb->publish(*msg_linear);
                        _pub_canusb->publish(*msg_angular);

                        flag_autonomous = false;
                    }
                }
            }
        }

        double SmartphoneGamepad::upcast(float value2)
        {
            //floatからdoubleにupcastするときに計算誤差を表示させないようにする。
            string a = std::to_string(value2);//floatをstringに変換
            double b = std::stod(a);//stringをdoubeに変換
            return b;
        }

    DualSense::DualSense(const rclcpp::NodeOptions &options) : DualSense("", options) {}
    DualSense::DualSense(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("controller_interface_node", name_space, options)
        {
        }
}
