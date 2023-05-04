#include "mcl_2d/mcl_2d_node.hpp"
#include "utilities/can_utils.hpp"
#include "rclcpp/time.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace mcl_2d{
    Mcl2D::Mcl2D(const rclcpp::NodeOptions &options) : Mcl2D("", options) {}

    Mcl2D::Mcl2D(const std::string &name_space, const rclcpp::NodeOptions &options)
    : rclcpp::Node("mcl_2d_node", name_space, options),
    mclocalizer(ament_index_cpp::get_package_share_directory("main_executor") +"/"+"config"+"/"+"mcl_2d") {

        _subscription_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan",
                _qos,
                std::bind(&Mcl2D::_subscriber_callback_laser, this, std::placeholders::_1)
        );

        _subscription_odom_linear = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_110",
                _qos,
                std::bind(&Mcl2D::_subscriber_callback_odom_linear, this, std::placeholders::_1)
        );
        _subscription_odom_angular = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
                "can_rx_111",
                _qos,
                std::bind(&Mcl2D::_subscriber_callback_odom_angular, this, std::placeholders::_1)
        );
        _subscription_initialize = this->create_subscription<std_msgs::msg::Empty>(
                "self_pose_initialize",
                _qos,
                std::bind(&Mcl2D::_subscriber_callback_initialize, this, std::placeholders::_1)
        );

        publisher_selfpose = this->create_publisher<geometry_msgs::msg::Vector3>("self_pose", _qos);

        /*パラメータ設定*/
        const auto numOfParticle = this->get_parameter("num_of_particle").as_int();
        const auto tf_array = this->get_parameter("tf_laser2robot").as_double_array();
        const auto pose_array = this->get_parameter("initial_pose").as_double_array();

        const float odomCovariance[6] = {
            static_cast<float>(this->get_parameter("odom_convariance.param1").as_double()),     // Rotation to Rotation
            static_cast<float>(this->get_parameter("odom_convariance.param2").as_double()),     // Translation to Rotation
            static_cast<float>(this->get_parameter("odom_convariance.param3").as_double()),     // Translation to Translation
            static_cast<float>(this->get_parameter("odom_convariance.param4").as_double()),     // Rotation to Translation
            static_cast<float>(this->get_parameter("odom_convariance.param5").as_double()),     // X
            static_cast<float>(this->get_parameter("odom_convariance.param6").as_double())      // Y
        };
        const Eigen::Matrix4f tf_laser2robot = tool::xyzrpy2eigen(tf_array[0],tf_array[1],tf_array[2],tf_array[3],tf_array[4],tf_array[5]);
        const Eigen::Matrix4f initial_pose = tool::xyzrpy2eigen(pose_array[0],pose_array[1],0,0,0,pose_array[2]);
        mclocalizer.setup(numOfParticle, odomCovariance, tf_laser2robot, initial_pose);

        // std::cout << tf_laser2robot << std::endl;
        RCLCPP_INFO(this->get_logger(), "init particle = %d",numOfParticle);
    }


    void Mcl2D::check_data(){

        while((vec_poses.size()!=0 && vec_lasers.size()!=0)){
            if(fabs(vec_poses_time[0] - vec_lasers_time[0])>0.01){

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

                auto self_pose = std::make_shared<geometry_msgs::msg::Vector3>();
                // self_pose->header.frame_id = "";
                // self_pose->header.stamp = observed_time;
                self_pose->x = mclocalizer.x;
                self_pose->y = mclocalizer.y;
                self_pose->z = mclocalizer.angle;

                publisher_selfpose->publish(*self_pose);
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
        observed_time = msg->header.stamp;
        this->check_data();

        // RCLCPP_INFO(this->get_logger(), "laser stamp : %lf", msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9);
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

        Eigen::Matrix4f eigenPose;
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        tf2::Matrix3x3 m(q);

        //試験補正
        // latest_pose.x += initial_pose.x;
        // latest_pose.y += initial_pose.y;
        // latest_pose.z += initial_pose.z;
        //ここまで

        eigenPose<< m[0][0], m[0][1], m[0][2], latest_pose.x,
                    m[1][0], m[1][1], m[1][2], latest_pose.y,
                    m[2][0], m[2][1], m[2][2], latest_pose.z,
                    0,0,0,1;

        tf2::Quaternion q_test;
        q_test.setRPY(M_PI, 0.0, 0.0);
        tf2::Matrix3x3 m_test(q_test);

        vec_poses.push_back(eigenPose);
        vec_poses_time.push_back(msg->header.stamp.sec + msg->header.stamp.nanosec*1e-9);
        observed_time = msg->header.stamp;
        check_data();
    }

    void Mcl2D::_subscriber_callback_initialize(const std_msgs::msg::Empty::SharedPtr msg){
        mclocalizer.odomInitialize();
    }

}  // namespace mcl_2d
