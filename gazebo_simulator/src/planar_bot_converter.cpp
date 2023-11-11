#include "gazebo_simulator/planar_bot_converter.hpp"
#include "utilities/can_utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

namespace gazebo_simulator {

PlanarBotConverter::PlanarBotConverter(const rclcpp::NodeOptions &options) : PlanarBotConverter("", options) {}

PlanarBotConverter::PlanarBotConverter(const std::string &name_space, const rclcpp::NodeOptions &options)
    : rclcpp::Node("planar_bot_converter", name_space, options){

    _subscription_velocity = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        _qos,
        std::bind(&PlanarBotConverter::callback_velocity, this, std::placeholders::_1)
    );
    _subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "gazebo_simulator/odom",
        _qos,
        std::bind(&PlanarBotConverter::callback_odom, this, std::placeholders::_1)
    );

    publisher_velocity = this->create_publisher<geometry_msgs::msg::Twist>("gazebo_simulator/cmd_vel", _qos);
    publisher_odom_linear = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_rx_110", _qos);
    publisher_odom_angular = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>("can_rx_111", _qos);
    this->declare_parameter<std::vector<double>>("initial_pose", {0.0, 0.0, 0.0});
    pose_array = this->get_parameter("initial_pose").as_double_array();
    // court = this->get_parameter("court_color").as_string();
    // if(court == "red"){
    //     pose_array[1]=-1.0*pose_array[1];
    // }
}

/*コールバック関数*/
//速度指令値を回転考慮して名前空間付きで発行
void PlanarBotConverter::callback_velocity(const geometry_msgs::msg::Twist::SharedPtr msg){
    auto msg_velocity = std::make_shared<geometry_msgs::msg::Twist>();
    const double c0 = cos(this->yaw);
    const double s0 = sin(this->yaw);
    msg_velocity->linear.x = c0*msg->linear.x + s0*msg->linear.y;
    msg_velocity->linear.y = -s0*msg->linear.x + c0*msg->linear.y;
    msg_velocity->angular.z = msg->angular.z*10.0;  //水平移動プラグインのバグを10倍して一時的に補正

    publisher_velocity->publish(*msg_velocity);

    // RCLCPP_INFO(this->get_logger(), "yaw : %lf", this->yaw);
}

//オドメトリをオイラー角にしてCANメッセージで名前空間なしで発行
void PlanarBotConverter::callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
    auto msg_linear = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_linear->canid = 0x110;
    msg_linear->candlc = 8;
    msg_linear->header = msg->header;
    auto msg_angular = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
    msg_angular->canid = 0x111;
    msg_angular->candlc = 4;
    msg_angular->header = msg->header;


    uint8_t _candata[8];
    float_to_bytes(_candata, static_cast<float>(msg->pose.pose.position.x - pose_array[0]));
    float_to_bytes(_candata+4, static_cast<float>(msg->pose.pose.position.y - pose_array[1]));
    for(int i=0; i<msg_linear->candlc; i++) msg_linear->candata[i] = _candata[i];

    // クオータニオンからオイラー角に変換
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // yaw角だけ使用
    this->yaw = yaw;    //速度指令に使う方向角を抽出
    float_to_bytes(_candata, static_cast<float>(yaw - pose_array[2]));
    for(int i=0; i<msg_angular->candlc; i++) msg_angular->candata[i] = _candata[i];

    publisher_odom_linear->publish(*msg_linear);
    publisher_odom_angular->publish(*msg_angular);
}

}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gazebo_simulator::PlanarBotConverter>());
  rclcpp::shutdown();
  return 0;
}