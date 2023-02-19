#include "controller_interface/controller_interface_node.hpp"

const VelPlannerLimit tvpl_stea(cfg::robot::move_pos_limit,cfg::robot::move_vel_limit,cfg::robot::move_acc_limit,cfg::robot::move_dec_limit);//stteaのlimitを参考にした
VelPlanner lin_x(tvpl_stea);
VelPlanner lin_y(tvpl_stea);
VelPlanner ang_z(tvpl_stea);

namespace controller_interface
{
    SmartphoneGamepad::SmartphoneGamepad(const rclcpp::NodeOptions &options) : SmartphoneGamepad("", options) {}
    SmartphoneGamepad::SmartphoneGamepad(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("controller_interface_node", name_space, options)
        {
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
            
            _pub_linear = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_angular = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_reset = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_emergency = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

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

            //RCLCPP_INFO(this->get_logger(), "flag:%d", msg->a);

            uint8_t _candata;

            _candata = (uint8_t)msg->s;
            for(int i=0; i<msg_reset->candlc; i++) msg_reset->candata[i] = _candata;

            _candata = (uint8_t)msg->g;
            for(int i=0; i<msg_emergency->candlc; i++) msg_emergency->candata[i] = _candata;

            _pub_reset->publish(*msg_reset);
            _pub_emergency->publish(*msg_emergency);
        }

        void SmartphoneGamepad::callback_scrn(const controller_interface_msg::msg::SubScrn::SharedPtr msg)
        {
            //上物インターフェイスノードと経路生成・計画ノードへ
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
            while(rclcpp::ok())
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

                lin_x.vel(analog_l_y);//unityとロボットにおける。xとyが違うので逆にしている。
                lin_y.vel(analog_l_x);
                ang_z.vel(analog_r_x);

                lin_x.cycle();
                lin_y.cycle();
                ang_z.cycle();

                RCLCPP_INFO(this->get_logger(), "vel_x:%f", lin_x.vel());

                float_to_bytes(_candata, (float)lin_x.vel() * max_linear_x);
                float_to_bytes(_candata+4, (float)lin_y.vel() * max_linear_y);
                for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata[i];

                float_to_bytes(_candata, (float)ang_z.vel() * max_angular_z);
                for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata[i];

                _pub_linear->publish(*msg_linear);
                _pub_angular->publish(*msg_angular);

                // auto end_time = std::chrono::steady_clock::now();
                // std::chrono::duration<double> elapsed_time = end_time - start_time;
                // RCLCPP_INFO(this->get_logger(), "Elapsed time: %f", elapsed_time.count());
            }
            
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
