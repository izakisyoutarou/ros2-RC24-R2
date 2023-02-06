#include "controller_interface/controller_interface_node.hpp"

const TraVelPlannerLimit tvpl_stea(cfg::robot::move_pos_limit,cfg::robot::move_vel_limit,cfg::robot::move_acc_limit,cfg::robot::move_dec_limit);//stteaのlimitを参考にした
TraVelPlanner lin_x(tvpl_stea);
TraVelPlanner lin_y(tvpl_stea);
TraVelPlanner ang_x(tvpl_stea);

namespace controller_interface
{

    ControllerInterface::ControllerInterface(const rclcpp::NodeOptions &options) : ControllerInterface("", options) {}
    ControllerInterface::ControllerInterface(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("controller_interface_node", name_space, options)
        {
            _sub_pad = this->create_subscription<controller_interface_msg::msg::SubPad>(
                "sub_pad",
                _qos,
                std::bind(&ControllerInterface::callback_pad, this, std::placeholders::_1)
            );

            _sub_scrn = this->create_subscription<controller_interface_msg::msg::SubScrn>(
                "sub_scrn",
                _qos,
                std::bind(&ControllerInterface::callback_scrn, this, std::placeholders::_1)
            );

            _pub_linear = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_angular = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_reset = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_emergency = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);

            const TraVelPlannerLimit tvpl_stea(cfg::robot::move_pos_limit,cfg::robot::move_vel_limit,cfg::robot::move_acc_limit,cfg::robot::move_dec_limit);//stteaのlimitを参考にした
            TraVelPlanner x(tvpl_stea);
            TraVelPlanner y(tvpl_stea);
        }

        void ControllerInterface::callback_pad(const controller_interface_msg::msg::SubPad::SharedPtr msg)
        {
            auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_linear->canid = 0x110;
            msg_linear->candlc = 8;

            auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_angular->canid = 0x111;
            msg_angular->candlc = 4;

            auto msg_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_reset->canid = 0x002;
            msg_reset->candlc = 1;

            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            this->anl_lft_x = roundoff(msg->anl_lft_x,1e-4);
            this->anl_lft_y = roundoff(msg->anl_lft_y,1e-4);
            this->anl_rgt_x = roundoff(msg->anl_rgt_x,1e-4);
            this->anl_rgt_y = roundoff(msg->anl_rgt_y,1e-4);

            lin_x.cycle();
            lin_y.cycle();
            ang_x.cycle();

            lin_x.vel(this->anl_lft_y);//unityとロボットにおける。xとyが違うので逆にしている。
            lin_y.vel(this->anl_lft_x);
            ang_x.vel(this->anl_rgt_x);

            RCLCPP_INFO(this->get_logger(), "flag:%d", can_error);

            if(can_error)
            {
                vel_lin_x = 0.0f;
                vel_lin_y = 0.0f;
                vel_ang_z = 0.0f;
            }
            else
            {
                vel_lin_x = (float)lin_x.vel() * max_linear_x;
                vel_lin_y = (float)lin_y.vel() * max_linear_y;
                vel_ang_z = (float)ang_x.vel() * max_angular_z;
            }


            uint8_t _candata[8];
            uint8_t flag;

            float_to_bytes(_candata, vel_lin_x);
            float_to_bytes(_candata+4, vel_lin_y);
            for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata[i];

            float_to_bytes(_candata, vel_ang_z);
            for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata[i];

            //flag = (uint8_t)msg->s;
            _candata[0] = (uint8_t)msg->s;
            for(int i=0; i<msg_reset->candlc; i++) msg_angular->candata[i] = _candata[i];

            //flag = (uint8_t)msg->g;
            _candata[0] = (uint8_t)msg->g;
            for(int i=0; i<msg_emergency->candlc; i++) msg_angular->candata[i] = _candata[i];

            _pub_linear->publish(*msg_linear);
            _pub_angular->publish(*msg_angular);
            _pub_reset->publish(*msg_reset);
            _pub_emergency->publish(*msg_emergency);
        }

        void ControllerInterface::callback_scrn(const controller_interface_msg::msg::SubScrn::SharedPtr msg)
        {
            //上物インターフェイスノードと経路生成・計画ノードへ


        }

        float ControllerInterface::roundoff(const float &value, const float &epsilon)
        {
            float ans = value;
            if(abs(ans) < epsilon) ans = 0.0;
            return ans;
        }
}
