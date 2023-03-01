#include "injection_param_calculator/injection_param_calculator.hpp"
using namespace utils;
namespace injection_param_calculator{
    InjectionParamCalculator::InjectionParamCalculator(const rclcpp::NodeOptions& options) : InjectionParamCalculator("",options){}
    InjectionParamCalculator::InjectionParamCalculator(const std::string &name_space, const rclcpp::NodeOptions &options)
     : Node("injection_param_calculator_node", name_space, options),
        ring_weight(get_parameter("ring_weight").as_double()),
        gravitational_acceleraStion(get_parameter("gravitational_acceleraStion").as_double()),
        air_resistance_coefficient(get_parameter("air_resistance_coefficient").as_double()),
        injection_length(get_parameter("injection_length").as_double()),
        injection_arm_length(get_parameter("injection_arm_length").as_double()),
        foundation_hight(get_parameter("foundation_hight").as_double()),
        angle_lim_max(get_parameter("angle_lim_max").as_double()),
        angle_lim_min(get_parameter("angle_lim_min").as_double()),
        first_velocity(get_parameter("first_velocity").as_double()),
        velocity_lim_max(get_parameter("velocity_lim_max").as_double()),
        eps(get_parameter("eps").as_double()),
        max_loop(get_parameter("max_loop").as_double())
        {

            _sub_right_injection = this->create_subscription<injection_interface_msg::msg::InjectionCommand>(
            "right_injection",
            _qos,
            std::bind(&InjectionParamCalculator::callback_right_injection,this,std::placeholders::_1)
        );
        _sub_left_injection = this->create_subscription<injection_interface_msg::msg::InjectionCommand>(
            "left_injection",
            _qos,
            std::bind(&InjectionParamCalculator::callback_left_injection,this,std::placeholders::_1)
        );
        
        _pub_right_injection = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx",_qos);
        _pub_left_injection = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_tx",_qos);
        RCLCPP_INFO(this->get_logger(),"create injection_param_calculator");
        }
    void InjectionParamCalculator::callback_right_injection(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg){
        auto msg_right_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_right_injection->canid = 0x120;
        msg_right_injection->candlc = 16;
        right_injection_command->distance = msg->distance;
        right_injection_command->height = msg->height;
        right_elevation = calculateElevation(right_injection_command);
        calculateVelocity(right_elevation);
    }
    void InjectionParamCalculator::callback_left_injection(const injection_interface_msg::msg::InjectionCommand::SharedPtr msg){
        auto msg_left_injection = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
        msg_left_injection->canid = 0x121;
        msg_left_injection->candlc = 16;
        left_injection_command->distance = msg->distance;
        left_injection_command->height = msg->height;
        left_elevation = calculateElevation(left_injection_command);
        calculateVelocity(left_elevation);
    }
    double InjectionParamCalculator::calculateElevation(injection_interface_msg::msg::InjectionCommand::SharedPtr msg){
        double elevation = atan2(msg->height,msg->distance);
        if(elevation > angle_bounds){
            return angle_lim_max;
        }
        else{
            return angle_choice;
        }
    }
    void InjectionParamCalculator::calculateVelocity(double elevation){
        int num_loop = 0;
        bool isAiming = false;
        bool isConvergenced = false;
        double old_velocity = first_velocity;
        if(elevation==right_elevation){
            while(!isAiming){
                double new_velocity = old_velocity - f(right_elevation,old_velocity)/diff(right_elevation,old_velocity);
                if(abs(new_velocity - old_velocity)<eps && 0<new_velocity && new_velocity < velocity_lim_max){
                    right_velocity = new_velocity;
                    isAiming=true;
                    isConvergenced=true;
                }
                old_velocity = new_velocity;
                num_loop++;
                if(num_loop>max_loop){
                    isConvergenced = false;
                    break;
                }
            }
            RCLCPP_INFO(this->get_logger(),"right_velocity: %lf",right_velocity);
        }
        else if(elevation==left_elevation){
            while(!isAiming){
                double new_velocity = old_velocity - f(left_elevation,old_velocity)/diff(left_elevation,old_velocity);
                if(abs(new_velocity-old_velocity)<eps && 0<new_velocity && new_velocity < velocity_lim_max){
                    left_velocity = new_velocity;
                    isAiming=true;
                    isConvergenced=true;
                }
                old_velocity = new_velocity;
                num_loop++;
                if(num_loop>max_loop){
                    isConvergenced = false;
                    break;
                }
            }
            RCLCPP_INFO(this->get_logger(),"left_velocity: %lf",left_velocity);
        }
    }
    double InjectionParamCalculator::f(double pitchCalculation,double v0){
        double c_sin = sin(dtor(pitchCalculation));
        double c_cos = cos(dtor(pitchCalculation));
        double c_tan = tan(dtor(pitchCalculation));
        double m = ring_weight;
        double g = gravitational_acceleraStion;
        double k = air_resistance_coefficient;
        double r = injection_length;
        double l = injection_arm_length;
        double y0 = foundation_hight;
        double x;
        double y;
        if(pitchCalculation==right_elevation){
            x = right_injection_command->direction - r*c_cos + l*c_sin;
            y = right_injection_command->height - (y0 + r*c_sin + l*c_cos);
        }
        else if(pitchCalculation==left_elevation){
            x = left_injection_command->direction - r*c_cos + l*c_sin;
            y = left_injection_command->height - (y0 + r*c_sin + l*c_cos);
        }
        return x*c_tan + m*g*x/(k*v0*c_cos) + m*m*g/(k*k)*log(abs(1-k*x/(m*v0*c_cos))) - y;
    }
    double InjectionParamCalculator::diff(double pitchCalculation,double v0){
        return (f(pitchCalculation, v0 + eps) - f(pitchCalculation, v0 - eps))/(2*eps);
    }
}