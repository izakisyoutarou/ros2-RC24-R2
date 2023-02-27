#include "controller_interface/controller_interface_node.hpp"

using namespace utils;

namespace controller_interface
{
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
        dtor(get_parameter("angular_max_dec").as_double()) )
        {
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
            _pub_linear = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_angular = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_restart = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_emergency = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

            //経路計画へpub
            _pub_route = this->create_publisher<controller_interface_msg::msg::RobotControll>("route",_qos);

            //上モノへpub
            _pub_tool = this->create_publisher<controller_interface_msg::msg::RobotControll>("tool",_qos);

            //gazebo_simulatorへ
            _pub_gazebo = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",_qos);

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
            auto msg_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_reset->canid = 0x002;
            msg_reset->candlc = 1;

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            auto msg_root = std::make_shared<controller_interface_msg::msg::RobotControll>();
            msg_root->emergency = msg->g;
            msg_root->manyual_swith = msg->r3;

            auto msg_tool = std::make_shared<controller_interface_msg::msg::RobotControll>();
            msg_tool->emergency = msg->g;
            msg_tool->manyual_swith = msg->r3;

            if(msg->r3)
            {
                if(mode == Mode::manual) mode = Mode::automatic;
                else mode = Mode::manual;
            }

            if(msg->s)
            {
                _candata_btn = 1;
                for(int i=0; i<msg_reset->candlc; i++) msg_reset->candata[i] = _candata_btn;
                _pub_restart->publish(*msg_reset);
            }

                
            _candata_btn = (uint8_t)msg->g;
            for(int i=0; i<msg_emergency->candlc; i++) msg_emergency->candata[i] = _candata_btn;
            
            _pub_emergency->publish(*msg_emergency);
            _pub_route->publish(*msg_root);
            _pub_tool->publish(*msg_tool);
        }

        void SmartphoneGamepad::callback_scrn(const controller_interface_msg::msg::SubScrn::SharedPtr msg)
        {
            //スマホのスクリーンボタンの押されたボタンを受信するcall_back
            //RCLCPP_INFO(this->get_logger(), "flag:%d", msg->p1);
        }

        void SmartphoneGamepad::callback_udp()
        {
            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = 0x110;
            msg_linear->candlc = 8;

            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = 0x111;
            msg_angular->candlc = 4;

            auto msg_gazebo = std::make_shared<geometry_msgs::msg::Twist>();
            while(rclcpp::ok())
            {
                if(mode == Mode::manual)
                {
                // auto start_time = std::chrono::steady_clock::now();

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

                analog_l_x = roundoff(analog_l_x,1e-4);
                analog_l_y = roundoff(analog_l_y,1e-4);
                analog_r_x = roundoff(analog_r_x,1e-4);
                analog_r_y = roundoff(analog_r_y,1e-4);

                velPlanner_linear_x.vel(upcast(analog_l_y));//unityとロボットにおける。xとyが違うので逆にしている。
                velPlanner_linear_y.vel(upcast(analog_l_x));
                velPlanner_angular_z.vel(upcast(analog_r_x));

                velPlanner_linear_x.cycle();
                velPlanner_linear_y.cycle();
                velPlanner_angular_z.cycle();

                //RCLCPP_INFO(this->get_logger(), "vel_x:%f", analog_l_y);

                float_to_bytes(_candata_joy, (float)velPlanner_linear_x.vel() * max_linear_x);
                float_to_bytes(_candata_joy+4, (float)velPlanner_linear_y.vel() * max_linear_y);
                for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata_joy[i];

                float_to_bytes(_candata_joy, (float)velPlanner_angular_z.vel() * max_angular_z);
                for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata_joy[i];

                msg_gazebo->linear.x = velPlanner_linear_x.vel();//gazebo_simulator
                msg_gazebo->linear.y = -velPlanner_linear_y.vel();
                msg_gazebo->angular.z = -velPlanner_angular_z.vel();

                _pub_linear->publish(*msg_linear);
                _pub_angular->publish(*msg_angular);
                _pub_gazebo->publish(*msg_gazebo);

                // auto end_time = std::chrono::steady_clock::now();
                // std::chrono::duration<double> elapsed_time = end_time - start_time;
                // RCLCPP_INFO(this->get_logger(), "Elapsed time: %f", elapsed_time.count());
                }
            }
            
        }

        double SmartphoneGamepad::upcast(float value2)
        {
            //floatからdoubleにupcastするときに計算誤差を表示させないようにする。
            string a = std::to_string(value2);//floatをstringに変換
            double b = std::stod(a);//stringをsoubeに変換
            return b;
        }

        float SmartphoneGamepad::roundoff(const float &value, const float &epsilon)
        {
            float ans = value;
            if(abs(ans) < epsilon) ans = 0.0;
            return ans;
        }

    
    DualSense::DualSense(const rclcpp::NodeOptions &options) : DualSense("", options) {}
    DualSense::DualSense(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("controller_interface_node", name_space, options)
        {
        }
}
