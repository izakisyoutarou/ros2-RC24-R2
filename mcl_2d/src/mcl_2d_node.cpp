#include "mcl_2d/mcl_2d_node.hpp"
#include "utilities/can_utils.hpp"
#include "rclcpp/time.hpp"

namespace mcl_2d{
    Mcl2D::Mcl2D(const rclcpp::NodeOptions &options) : Mcl2D("", options) {}

    Mcl2D::Mcl2D(const std::string &name_space, const rclcpp::NodeOptions &options)
    : rclcpp::Node("mcl_2d_node", name_space, options) {

        _subscription_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan",
                _qos,
                std::bind(&Mcl2D::_subscriber_callback_laser, this, std::placeholders::_1)
        );

        _subscription_odom_linear = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_100",
                _qos,
                std::bind(&Mcl2D::_subscriber_callback_odom_linear, this, std::placeholders::_1)
        );
        _subscription_odom_angular = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_101",
                _qos,
                std::bind(&Mcl2D::_subscriber_callback_odom_angular, this, std::placeholders::_1)
        );
    }


    void Mcl2D::check_data(){
        // RCLCPP_INFO(this->get_logger(), "pose size:%lf  laser size:%lf", vec_poses.size(), vec_lasers.size());
        RCLCPP_INFO(this->get_logger(), "POSE x:%lf  y:%lf", mclocalizer.position_x, mclocalizer.position_y);
        while((vec_poses.size()!=0 && vec_lasers.size()!=0)){
            if(fabs(vec_poses_time[0] - vec_lasers_time[0])>0.1){

                if(vec_poses_time[0]>vec_lasers_time[0]){
                    vec_lasers.erase(vec_lasers.begin());
                    vec_lasers_time.erase(vec_lasers_time.begin());
                }
                else{
                    vec_poses.erase(vec_poses.begin());
                    vec_poses_time.erase(vec_poses_time.begin());
                }
            }
            else{
                mclocalizer.updateData(vec_poses[0],vec_lasers[0]);
                vec_lasers.erase(vec_lasers.begin());
                vec_lasers_time.erase(vec_lasers_time.begin());
                vec_poses.erase(vec_poses.begin());
                vec_poses_time.erase(vec_poses_time.begin());
            }
        }
    }

    void Mcl2D::_subscriber_callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg){
        int scanQuantity =((msg->angle_max)-(msg->angle_min))/(msg->angle_increment)+1;
        Eigen::Matrix4Xf eigenLaser = Eigen::Matrix4Xf::Ones(4, 1);
        int scanEffective = 0;

        for(int i=0;i<scanQuantity;i++){
            float dist = msg->ranges[i];
            if(dist > 1 && dist < 10){
                scanEffective++;
                eigenLaser.conservativeResize(4,scanEffective);
                eigenLaser(0,scanEffective-1) =  dist * cos(msg->angle_min + ( msg->angle_increment * i));
                eigenLaser(1,scanEffective-1) =  dist * sin(msg->angle_min + ( msg->angle_increment * i));
                eigenLaser(2,scanEffective-1) =  0;
                eigenLaser(3,scanEffective-1) =  1;
            }
        }
        vec_lasers.push_back(eigenLaser);
        vec_lasers_time.push_back(msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9);
        this->check_data();

        RCLCPP_INFO(this->get_logger(), "laser stamp : %lf", msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9);
    }


    void Mcl2D::_subscriber_callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
        uint8_t _candata[8];
        for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
        latest_pose.x = (double)bytes_to_float(_candata);
        latest_pose.y = (double)bytes_to_float(_candata+4);
    }

    void Mcl2D::_subscriber_callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
        uint8_t _candata[8];
        for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
        double yaw = (double)bytes_to_float(_candata);

        // RCLCPP_INFO(this->get_logger(), "x:%lf  y:%lf  a:%lf", latest_pose.x, latest_pose.y, yaw);

        Eigen::Matrix4f eigenPose;
        // tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        tf2::Matrix3x3 m(q);
        // eigenPose<< m[0][0], m[0][1], m[0][2], latest_pose.x,
        //             m[1][0], m[1][1], m[1][2], latest_pose.y,
        //             m[2][0], m[2][1], m[2][2], latest_pose.z,
        //             0,0,0,1;
        eigenPose<< m[0][0], m[0][1], m[0][2], latest_pose.x,
                    m[1][0], m[1][1], m[1][2], latest_pose.y,
                    m[2][0], m[2][1], m[2][2], latest_pose.z,
                    0,0,0,1;

        tf2::Quaternion q_test;
        q_test.setRPY(M_PI, 0.0, 0.0);
        tf2::Matrix3x3 m_test(q_test);
        // RCLCPP_INFO(this->get_logger(), "\n%lf  %lf  %lf\n %lf  %lf  %lf\n %lf  %lf  %lf\n"
        // ,   m_test[0][0],m_test[0][1],m_test[0][2],
        //     m_test[1][0],m_test[1][1],m_test[1][2],
        //     m_test[2][0],m_test[2][1],m_test[2][2]);

        vec_poses.push_back(eigenPose);
        vec_poses_time.push_back(msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9);
        check_data();
    }

}  // namespace mcl_2d
