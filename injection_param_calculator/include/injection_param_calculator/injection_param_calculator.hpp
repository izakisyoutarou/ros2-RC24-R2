#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "socketcan_interface_msg/msg/socketcan_if.hpp"
#include "injection_interface_msg/msg/injection_command.hpp"
#include "utilities/utils.hpp"
#include "injection_param_calculator/my_visibility.h"

namespace injection_param_calculator{
    class InjectionParamCalculator : public rclcpp::Node{
        public:
            INJECTION_PARAM_CALCULATOR_PUBLIC
            explicit InjectionParamCalculator(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

            INJECTION_PARAM_CALCULATOR_PUBLIC
            explicit InjectionParamCalculator(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        private:
            rclcpp::Subscription<injection_interface_msg::msg::InjectionCommand>::SharedPtr _sub_right_injection;
            rclcpp::Subscription<injection_interface_msg::msg::InjectionCommand>::SharedPtr _sub_left_injection;
            rclcpp::QoS _qos = rclcpp::QoS(10);

            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_right_injection;
            rclcpp::Publisher<socketcan_interface_msg::msg::SocketcanIF>::SharedPtr _pub_left_injection;

            void callback_right_injection(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg);
            void callback_left_injection(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg);
            
            double f(double pitchCalculation,double v0);
            double diff(double pitchCalculation,double v0);
            double calculateElevation(injection_interface_msg::msg::InjectionCommand::SharedPtr msg);
            void calculateVelocity(double elevation);

            injection_interface_msg::msg::InjectionCommand::SharedPtr left_injection_command;
            injection_interface_msg::msg::InjectionCommand::SharedPtr right_injection_command;

            double right_velocity;
            double left_velocity;
            double right_elevation;
            double left_elevation;

            double ring_weight;   //リングの質量
            double gravitational_acceleraStion;   //重力加速度
            double air_resistance_coefficient;    //空気抵抗係数
            double injection_length;  //射出機構の長さ
            double injection_arm_length;  //射出機構の腕の長さ
            double foundation_hight;  //射出機構の腕の長さ
            double first_velocity;  //初速度の初期値

            double angle_lim_max; //最大射角
            double angle_lim_min; //最小射角
            double velocity_lim_max; //最大初速度
            double angle_choice;    //リングが入りやすい角度
            double angle_bounds;    //角度の境界
            double eps;
            double max_loop; //  ニュートン法の最大繰り返し数
    };
}