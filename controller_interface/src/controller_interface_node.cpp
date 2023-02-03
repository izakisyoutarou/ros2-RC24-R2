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

            // _sub_scrn = this->create_subscription<controller_interface_msg::msg::SubScrn>(
            //     "sub_scrn",
            //     _qos,
            //     std::bind(&ControllerInterface::callback_scrn, this, std::placeholders::_1)
            // );

            _sub_scrn = this->create_subscription<std_msgs::msg::String>(
                "sub_scrn",
                _qos,
                std::bind(&ControllerInterface::callback_scrn, this, std::placeholders::_1)
            );

            _pub_linear = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_angular = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx", _qos);
            _pub_reset = this->create_publisher<std_msgs::msg::Empty>("can_tx", _qos);
            _pub_emergency = this->create_publisher<std_msgs::msg::Empty>("can_tx", _qos);

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

            // auto msg_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            // msg_reset->canid = 0x002;
            // msg_reset->candlc = 1;

            // auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            // msg_emergency->canid = 0x000;
            // msg_emergency->candlc = 1;

            this->anl_lft_x = roundoff(msg->anl_lft_x,1e-4);
            this->anl_lft_y = roundoff(msg->anl_lft_y,1e-4);
            this->anl_rgt_x = roundoff(msg->anl_rgt_x,1e-4);
            this->anl_rgt_y = roundoff(msg->anl_rgt_y,1e-4);

            lin_x.cycle();
            lin_y.cycle();
            ang_x.cycle();

            lin_x.vel(msg->anl_lft_y);
            lin_y.vel(msg->anl_lft_x);
            ang_x.vel(msg->anl_rgt_x);

            uint8_t _candata[8];

            float_to_bytes(_candata, (float)lin_x.vel());
            float_to_bytes(_candata+4, (float)lin_y.vel());
            for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata[i];

            //float_to_bytes(_candata, roundoff(msg->anl_rgt_x,1e-4)*max_angular_z);
            float_to_bytes(_candata, (float)ang_x.vel());
            for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata[i];

            _pub_linear->publish(*msg_linear);
            _pub_angular->publish(*msg_angular);
        }

        // void ControllerInterface::callback_scrn(const controller_interface_msg::msg::SubScrn::SharedPtr msg)
        // {

        // }

        void ControllerInterface::callback_scrn(const controller_interface_msg::msg::SubScrn::SharedPtr msg)
        {
            //上物インターフェイス
            RCLCPP_INFO(this->get_logger(), "Hello");
            //RCLCPP_INFO(this->get_logger(), "%s", msg->scrn);
        }

        void ControllerInterface::callback_reset(const std_msgs::msg::Empty::SharedPtr msg)
        {
            auto msg_reset = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_reset->canid = 0x002;
            msg_reset->candlc = 1;

            uint8_t _candata = msg->structure_needs_at_least_one_member;
            msg_reset->candata[0] = _candata;
        }

        void ControllerInterface::callback_emergency(const std_msgs::msg::Empty::SharedPtr msg)
        {
            auto msg_emergency = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
            msg_emergency->canid = 0x000;
            msg_emergency->candlc = 1;

            uint8_t _candata = msg->structure_needs_at_least_one_member;
            msg_emergency->candata[0] = _candata;
        }

        float ControllerInterface::roundoff(const float &value, const float &epsilon)
        {
            float ans = value;
            if(abs(ans) < epsilon) ans = 0.0;
            return ans;
        }
}
