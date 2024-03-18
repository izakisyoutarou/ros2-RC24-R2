#include "detection_interface/detection_interface_node.hpp"

namespace detection_interface
{
    DetectionInterface::DetectionInterface(const rclcpp::NodeOptions &options) : DetectionInterface("", options) {}
    DetectionInterface::DetectionInterface(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("detection_interface_node", name_space, options),
        str_self_pose_range(get_parameter("str_self_pose_range").as_double_array()),
        siro_self_pose_range(get_parameter("siro_self_pose_range").as_double_array()),
        str_range_point1(get_parameter("str_range_point1").as_double_array()),
        str_range_point2(get_parameter("str_range_point2").as_double_array()),
        str_range_x_C3orC5(get_parameter("str_range_x_C3orC5").as_double_array()),
        str_ball_range_y(get_parameter("str_ball_range_y").as_double()),
        siro_ball_range_y(get_parameter("siro_ball_range_y").as_double_array()),
        siro_ball_range_x(get_parameter("siro_ball_range_x").as_double_array())
        {
            //yolox_ros_cppのrealsenseから
            _sub_realsense = this->create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>(
                "yolox/realsense",
                _qos,
                std::bind(&DetectionInterface::callback_realsense, this, std::placeholders::_1)
            );

            //yolox_ros_cppのc1から
            _sub_c1 = this->create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>(
                "yolox/c1",
                _qos,
                std::bind(&DetectionInterface::callback_c1, this, std::placeholders::_1)
            );

            //ransacから
            _sub_self_pose = this->create_subscription<geometry_msgs::msg::Vector3>(
                "self_pose",
                _qos,
                std::bind(&DetectionInterface::callback_self_pose, this, std::placeholders::_1)
            );

            _sub_base_control = this->create_subscription<controller_interface_msg::msg::BaseControl>(
                "base_control",
                _qos,
                std::bind(&DetectionInterface::callback_base_control, this, std::placeholders::_1)
            );

            //sequncerから
            _sub_now_sequence = this->create_subscription<std_msgs::msg::String>(
                "now_sequence",
                _qos,
                std::bind(&DetectionInterface::callback_now_sequence, this, std::placeholders::_1)
            );

            //splinwから
            _sub_way_point = this->create_subscription<std_msgs::msg::String>(
                "way_point",
                _qos,
                std::bind(&DetectionInterface::callback_way_point, this, std::placeholders::_1)
            );

            _sub_depth_ = image_transport::create_subscription(
                this, "/camera/camera/depth/image_rect_raw",
                std::bind(&DetectionInterface::depthImageCallback, this, std::placeholders::_1),
                "raw"
            );
        
            //sequncerへ
            _pub_collection_point = this->create_publisher<std_msgs::msg::String>("collection_point", _qos);
            _pub_siro_param = this->create_publisher<detection_interface_msg::msg::SiroParam>("siro_param", _qos);
            _pub_front_ball = this->create_publisher<std_msgs::msg::Bool>("front_ball", _qos);
            _pub_ball_coordinate = this->create_publisher<geometry_msgs::msg::Vector3>("ball_coordinate", _qos);

            //arm_param_caluculatorへ
            _pub_arm_param = this->create_publisher<detection_interface_msg::msg::ArmParam>("arm_param", _qos);

        }

        void DetectionInterface::callback_c1(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg){
            auto msg_collection_point = std::make_shared<std_msgs::msg::String>();
            auto msg_siro_param = std::make_shared<detection_interface_msg::msg::SiroParam>();

            int center_x;
            int center_y;

            float probabillity;
            uint16_t xmin;
            uint16_t ymin;
            uint16_t xmax;
            uint16_t ymax;
            uint16_t id;
            uint16_t img_width;
            uint16_t img_height;
            uint32_t center_dist;
            std::string class_id;
            //boxeesの配列からboxの内容を取り出し
            for (const auto& box : msg->bounding_boxes) {
                probabillity = box.probability;
                xmin = box.xmin;
                ymin = box.ymin;
                xmax = box.xmax;
                ymax = box.ymax;
                id = box.id;
                img_width = box.img_width;
                img_height = box.img_height;
                center_dist = box.center_dist;
                class_id = box.class_id;
            }

            center_x = (xmax + xmin) / 2;
            center_y = (ymax + ymin) / 2;

            //ひし形モードのときに、ひし形に向かっているとき
            if(now_sequence == "storage"){
                if(is_self_pose_range_x_str && is_self_pose_range_y_str && is_self_pose_range_z_str){
                    bool is_collection_C5;

                    is_collection_C5 = bounday_line(center_x, center_y);
                    
                    if(is_collection_C5) msg_collection_point->data = "c4";
                    else msg_collection_point->data = "c3";

                    std::cout << msg_collection_point->data << std::endl;

                    _pub_collection_point->publish(*msg_collection_point);
                }
            }
            //モード問わず、サイロに向かっているとき
            if(is_self_pose_range_x_siro && is_self_pose_range_y_siro && is_self_pose_range_z_siro){
                int siro_num;
                int i;

                //ボールの段数
                if(center_y < siro_ball_range_y[0]) siro_num = 1;
                else if(center_y > siro_ball_range_y[0] && center_y < siro_ball_range_y[1]) siro_num = 2;
                else if(center_y > siro_ball_range_y[1]) siro_num = 3;

                //どのサイロか
                if(siro_num != 0){
                    if(center_x < siro_ball_range_x[0]) siro_num += 0;
                    else if(center_x > siro_ball_range_x[0] && center_x < siro_ball_range_x[1]) siro_num += 3;
                    else if(center_x > siro_ball_range_x[1] && center_x < siro_ball_range_x[2]) siro_num += 6;
                    else if(center_x > siro_ball_range_x[2] && center_x < siro_ball_range_x[3]) siro_num += 9;
                    else if(center_x > siro_ball_range_x[3]) siro_num += 12;
                }

                // RCLCPP_INFO(this->get_logger(), "%d", siro_num);
                msg_siro_param->siro_param[0] = to_string(siro_num);

                if(class_id == "redball") msg_siro_param->siro_param[1] = 'R';
                else if(class_id == "blueball") msg_siro_param->siro_param[1] = 'B';
                else if(class_id == "yelloball") msg_siro_param->siro_param[1] = 'Y';
                
                // for(i = 0;i < 15;i++){
                //     if(siro_num == i){
                //         //どの色か
                //         if(class_id == "redball") msg_siro_param->siro_param[i] = 'R';
                //         else if(class_id == "blueball") msg_siro_param->siro_param[i] = 'B';
                //         else if(class_id == "yelloball") msg_siro_param->siro_param[i] = 'Y';
                //         // char test1 = msg_siro_param->siro_param[i];
                //         // std::cout << test1 << std::endl;
                //     }
                // }

                _pub_siro_param->publish(*msg_siro_param);
            }
        }

        void DetectionInterface::callback_realsense(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg){
            if(now_sequence == "storage" || now_sequence == "collect")
            {
                auto msg_collection_point = std::make_shared<std_msgs::msg::String>();
                auto msg_front_ball = std::make_shared<std_msgs::msg::Bool>();
                auto msg_ball_coordinate = std::make_shared<geometry_msgs::msg::Vector3>();

                int center_x;
                int center_y;

                float probabillity;
                uint16_t xmin;
                uint16_t ymin;
                uint16_t xmax;
                uint16_t ymax;
                uint16_t id;
                uint16_t img_width;
                uint16_t img_height;
                uint32_t center_dist;
                std::string class_id;

                for (const auto& box : msg->bounding_boxes) {
                    probabillity = box.probability;
                    xmin = box.xmin;
                    ymin = box.ymin;
                    xmax = box.xmax;
                    ymax = box.ymax;
                    id = box.id;
                    img_height = box.img_height;
                    class_id = box.class_id;
                }

                center_x = (xmax + xmin) / 2;
                center_y = (ymax + ymin) / 2;

                // center_dist = cv_image_.at<uint32_t>(center_y, center_x);
                // RCLCPP_INFO(this->get_logger(), "%d", center_dist);

                Vector3d test = ct.Rx_Ry_Rz(center_x, center_y, /*(double)center_dist*/200, pose);

                msg_ball_coordinate->x = test[0];
                msg_ball_coordinate->y = test[1];
                msg_ball_coordinate->z = test[2];
                // RCLCPP_INFO(this->get_logger(), "%f", center_x);

                if(way_point == "c3"){
                    if(center_x < str_range_x_C3orC5[0]) msg_collection_point->data = "ST0";
                    else if(center_x > str_range_x_C3orC5[0] && center_x < str_range_x_C3orC5[1]) msg_collection_point->data = "ST1";
                    else if(center_x > str_range_x_C3orC5[1] && center_x < str_range_x_C3orC5[2]) msg_collection_point->data = "ST2";
                    else if(center_x > str_range_x_C3orC5[2]) msg_collection_point->data = "ST3";
                    std::cout << msg_collection_point->data << std::endl;
                    way_point = "";
                }
                else if(way_point == "c6"){
                    if(center_x < str_range_x_C3orC5[0]) msg_collection_point->data = "ST7";
                    else if(center_x > str_range_x_C3orC5[0] && center_x < str_range_x_C3orC5[1]) msg_collection_point->data = "ST6";
                    else if(center_x > str_range_x_C3orC5[1] && center_x < str_range_x_C3orC5[2]) msg_collection_point->data = "ST5";
                    else if(center_x > str_range_x_C3orC5[2]) msg_collection_point->data = "ST4";
                    way_point = "";
                }

                if(!way_point.empty()){
                    if(center_y < str_ball_range_y) msg_front_ball->data = true;
                    else msg_front_ball->data = false;
                }
            
                way_point = "";
                _pub_collection_point->publish(*msg_collection_point);
                _pub_front_ball->publish(*msg_front_ball);
                _pub_ball_coordinate->publish(*msg_ball_coordinate);
            }
        }

        void DetectionInterface::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &ptr){
            std::shared_ptr<cv_bridge::CvImage> bridge_ = cv_bridge::toCvCopy(ptr, sensor_msgs::image_encodings::TYPE_16UC1);
            cv_image_ = bridge_->image;
        }

        void DetectionInterface::callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
            is_self_pose_range_x_str = (msg->x > str_self_pose_range[0] && msg->x < str_self_pose_range[1]) ? true : false;
            is_self_pose_range_y_str = (msg->y > str_self_pose_range[2] && msg->y < str_self_pose_range[3]) ? true : false;
            is_self_pose_range_z_str = (msg->z > str_self_pose_range[4] && msg->z < str_self_pose_range[5]) ? true : false;
            is_self_pose_range_x_siro = (msg->x > siro_self_pose_range[0] && msg->x < siro_self_pose_range[1]) ? true : false;
            is_self_pose_range_y_siro = (msg->y > siro_self_pose_range[2] && msg->y < siro_self_pose_range[3]) ? true : false;
            is_self_pose_range_z_siro = (msg->z > siro_self_pose_range[4] && msg->z < siro_self_pose_range[5]) ? true : false;
            pose[0] = msg->x;
            pose[1] = msg->y;
            pose[2] = msg->z;
        }

        void DetectionInterface::callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
            if(msg->is_restart){
                way_point = "";
                now_sequence = "";
            }
        }

        void DetectionInterface::callback_now_sequence(const std_msgs::msg::String::SharedPtr msg){
            now_sequence = msg->data;
        }

        void DetectionInterface::callback_way_point(const std_msgs::msg::String::SharedPtr msg){
            way_point = msg->data;
            // std::cout << way_point << std::endl;
        }
        //坂上からひし形を見たときの斜めの線
        bool DetectionInterface::bounday_line(int x, int y){
            float a;
            float b;
            float c;
            float dx;
            float dy;
            float calculation;
            bool is_collection_C5;

            dx = str_range_point2[0] - str_range_point1[0];
            dy = str_range_point2[1] - str_range_point1[1];

            a = -(str_range_point2[1] - str_range_point1[1]);
            b = str_range_point2[0] - str_range_point1[0];
            c = -(str_range_point2[1] - str_range_point1[1]) * str_range_point2[0] - str_range_point2[1] * (str_range_point2[1] - str_range_point1[1]);

            calculation = (a*x + b*y + c) / sqrt(a*a + b*b);

            is_collection_C5 = (calculation >= 0) ? true : false;

            return is_collection_C5;
        }

}