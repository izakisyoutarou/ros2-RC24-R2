#include "detection_interface/detection_interface_node.hpp"

namespace detection_interface
{
    DetectionInterface::DetectionInterface(const rclcpp::NodeOptions &options) : DetectionInterface("", options) {}
    DetectionInterface::DetectionInterface(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("detection_interface_node", name_space, options),
        str_self_pose_range(get_parameter("str_self_pose_range").as_double_array()),
        siro_self_pose_range(get_parameter("siro_self_pose_range").as_double_array()),

        str_range_point1_blue(get_parameter("str_range_point1_blue").as_double_array()),
        str_range_point2_blue(get_parameter("str_range_point2_blue").as_double_array()),
        str_range_point1_red(get_parameter("str_range_point1_red").as_double_array()),
        str_range_point2_red(get_parameter("str_range_point2_red").as_double_array()),

        str_range_point1_1_C2(get_parameter("str_range_point1_1_C2").as_double_array()),
        str_range_point1_2_C2(get_parameter("str_range_point1_2_C2").as_double_array()),
        str_range_point2_1_C2(get_parameter("str_range_point2_1_C2").as_double_array()),
        str_range_point2_2_C2(get_parameter("str_range_point2_2_C2").as_double_array()),
        str_range_point3_1_C2(get_parameter("str_range_point3_1_C2").as_double_array()),
        str_range_point3_2_C2(get_parameter("str_range_point3_2_C2").as_double_array()),
        str_range_point4_1_C2(get_parameter("str_range_point4_1_C2").as_double_array()),
        str_range_point4_2_C2(get_parameter("str_range_point4_2_C2").as_double_array()),

        str_range_x_C3orC5(get_parameter("str_range_x_C3orC5").as_double_array()),
        str_range_y_C3orC5(get_parameter("str_range_y_C3orC5").as_double()),

        str_range_x_ST(get_parameter("str_range_x_ST").as_double_array()),
        str_range_y_ST(get_parameter("str_range_y_ST").as_double_array()),
        
        siro_ball_range_y(get_parameter("siro_ball_range_y").as_double_array()),
        siro_ball_range_x(get_parameter("siro_ball_range_x").as_double_array()),

        court_color(get_parameter("court_color").as_string())
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
                rclcpp::SensorDataQoS(),
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
            // RCLCPP_INFO(this->get_logger(), "%d", n);
            time_start = chrono::system_clock::now();
            auto msg_collection_point = std::make_shared<std_msgs::msg::String>();
            auto msg_siro_param = std::make_shared<detection_interface_msg::msg::SiroParam>();

            std::vector<std::string> class_id;
            std::vector<int> ymax;
            std::vector<int> center_x;
            std::vector<int> center_y;
            std::vector<int> bbounbox_size;
            
            std::vector<std::string> c3_or_c4;

            std::vector<std::string> ball_color;
            std::vector<int> before_ball_place;
            std::vector<int> after_ball_place;

            int siro_num = 0;

            //boxeesの配列からboxの内容を取り出し
            for (const auto& box : msg->bounding_boxes) {
                class_id.push_back(box.class_id);
                ymax.push_back(box.ymax);

                int center_x_value = static_cast<int>((box.xmax + box.xmin) / 2);
                int center_y_value = static_cast<int>((box.ymax + box.ymin) / 2);
                int size = static_cast<int>((box.xmax - box.xmin) * (box.ymax - box.ymin));
                center_x.push_back(center_x_value);
                center_y.push_back(center_y_value);
                bbounbox_size.push_back(size);
            }

            // RCLCPP_INFO(this->get_logger(), "x@%d", center_x[0]);
            // RCLCPP_INFO(this->get_logger(), "y@%d", center_y[0]);

                //ひし形モードのときに、ひし形に向かっているとき
            if(now_sequence == "storage"){
                // if(is_self_pose_range_x_str && is_self_pose_range_y_str && is_self_pose_range_z_str){
                if(way_point == "c2"){
                    if(storage_flag){//認識区間内で一回だけc3 or c4 or ""を送るようにする。サイロに向かうところで再度tureになる
                        for (size_t i = 0; i < center_x.size(); ++i) {
                            if(rhombus_inside(center_x[i], center_y[i])){//坂上からc1で見たときにひし形の中にある
                                bool is_collection_C4 = false;

                                if(court_color == "blue") is_collection_C4 = bounday_line(center_x[i], center_y[i], str_range_point1_blue, str_range_point2_blue);
                                else is_collection_C4 = bounday_line(center_x[i], center_y[i], str_range_point1_red, str_range_point2_red);

                                if(is_collection_C4) c3_or_c4.push_back("c4");
                                else c3_or_c4.push_back("c3");
                            }
                        }
                        auto c3 = std::find(c3_or_c4.begin(), c3_or_c4.end(), "c3");
                        auto c4 = std::find(c3_or_c4.begin(), c3_or_c4.end(), "c4");

                        if(c3 != c3_or_c4.end())msg_collection_point->data = "c3";
                        else if(c4 != c3_or_c4.end())msg_collection_point->data = "c4";
                        _pub_collection_point->publish(*msg_collection_point);

                        storage_flag = false;
                    }
                }
            }

                //モード問わず、サイロに向かっているとき
                // if(is_self_pose_range_x_siro && is_self_pose_range_y_siro && is_self_pose_range_z_siro){
                if(way_point == "c1"){
                    storage_flag = true;//c1カメラのストレージゾーン認識のflag
                    for (size_t i = 0; i < center_x.size(); ++i) {
                        RCLCPP_INFO(this->get_logger(), "ymax@%d", ymax[i]);
                        siro_num = 0;
                        // if(way_point == "C1") siro_num = 3;

                        //ボールの段数
                        if(center_y[i] < siro_ball_range_y[0]) siro_num += 1;
                        else if(center_y[i] > siro_ball_range_y[0] && center_y[i] < siro_ball_range_y[1]) siro_num += 2;
                        else if(ymax[i] > siro_ball_range_y[1] && ymax[i] < siro_ball_range_y[2]) siro_num += 3;

                        //どのサイロか
                        if(siro_num != 0){
                            if(court_color == "blue"){
                                if(center_x[i] < siro_ball_range_x[0]) siro_num += 12;
                                else if(center_x[i] > siro_ball_range_x[0] && center_x[i] < siro_ball_range_x[1]) siro_num += 9;
                                else if(center_x[i] > siro_ball_range_x[1] && center_x[i] < siro_ball_range_x[2]) siro_num += 6;
                                else if(center_x[i] > siro_ball_range_x[2] && center_x[i] < siro_ball_range_x[3]) siro_num += 3;
                                else if(center_x[i] > siro_ball_range_x[3]) siro_num += 0;
                            }
                            else{
                                if(center_x[i] < siro_ball_range_x[0]) siro_num += 0;
                                else if(center_x[i] > siro_ball_range_x[0] && center_x[i] < siro_ball_range_x[1]) siro_num += 3;
                                else if(center_x[i] > siro_ball_range_x[1] && center_x[i] < siro_ball_range_x[2]) siro_num += 6;
                                else if(center_x[i] > siro_ball_range_x[2] && center_x[i] < siro_ball_range_x[3]) siro_num += 9;
                                else if(center_x[i] > siro_ball_range_x[3]) siro_num += 12;
                            }
                            before_ball_place.push_back(siro_num);

                            if(class_id[i] == "redball") ball_color.push_back("R");
                            else if(class_id[i] == "blueball") ball_color.push_back("B");
                        }


                        
                    }
                }

            //サイロ方面の検出が行われると入るところ
            if(before_ball_place.size() != 0){
                // for (int z = 0; z < before_ball_place.size(); ++z) {
                //     RCLCPP_INFO(this->get_logger(), "まえ%d@%d,サイズ@%d", z, before_ball_place[z],bbounbox_size[z]);
                // }

                //同じ領域にボールが複数存在している場合、バウンディングボックスのサイズが大きい方を採用する。それのところ
                for (int a = 0; a < before_ball_place.size(); ++a){
                    for (int b = a + 1; b < before_ball_place.size(); ++b){
                        if (before_ball_place[a] == before_ball_place[b]) {
                            if(bbounbox_size[a] > bbounbox_size[b]){
                                bbounbox_size.erase(bbounbox_size.begin() + b);
                                before_ball_place.erase(before_ball_place.begin() + b);
                                ball_color.erase(ball_color.begin() + b);
                                --b;
                            }
                            else{
                                bbounbox_size.erase(bbounbox_size.begin() + a);
                                before_ball_place.erase(before_ball_place.begin() + a);
                                ball_color.erase(ball_color.begin() + a);
                                --a;
                                break;
                            }
                        }
                    }
                }

                //例えば、サイロの1段目が空いている状態で2段目が埋まったとき、2段目のボールは空中に浮いていることになり、正しくない。
                //そういうときに1段目が埋まって、2段目が埋まったように修正するためのところ
                for(int i = 0; i < before_ball_place.size(); ++i){
                    bool stage_three = false;
                    bool stage_two = false;
                    int ball_height = 0;
                    int ball_width = 0;

                    ball_width = before_ball_place[i] / 3;
                    ball_height = before_ball_place[i] % 3;

                    if(ball_height != 0){
                        for(int d = 0; d < 3; ++d){//ball_heightの可能性として1%3と2%3と3%3なのでd < 3
                            if(ball_height == d){//ballが他に存在しているか。
                                for(size_t e = 0; e < before_ball_place.size(); ++e){
                                    if(before_ball_place[e] == ball_width * 3 + 3) stage_three = true;
                                    if(before_ball_place[e] == ball_width * 3 + 2) stage_two = true;
                                }
                                if(stage_three && stage_two) siro_num = ball_width * 3 + 1;
                                else if(stage_three && !stage_two) siro_num = ball_width * 3 + 2;
                                else if(!stage_three && stage_two) siro_num = ball_width * 3 + 3;
                                else if(!stage_three && !stage_two) siro_num = ball_width * 3 + 3;
                            }
                        }
                    }
                    else{
                        siro_num = ball_width * 3;
                    }
                    after_ball_place.push_back(siro_num);
                }

                // for (int z = 0; z < after_ball_place.size(); ++z) {
                //     RCLCPP_INFO(this->get_logger(), "あと%d@%d,サイズ@%d", z, after_ball_place[z],bbounbox_size[z]);
                // }

                for (int i = 0; i < after_ball_place.size(); ++i) {
                    msg_siro_param->ball_color[after_ball_place[i] - 1] = ball_color[i];
                }

                _pub_siro_param->publish(*msg_siro_param);
            }
            
            

            time_end = chrono::system_clock::now();
            // RCLCPP_INFO(this->get_logger(), "scan time->%d[ms]", chrono::duration_cast<chrono::milliseconds>(time_end-time_start).count());
        }

        void DetectionInterface::callback_realsense(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg){
            if(now_sequence == "storage" || now_sequence == "collect"){
                auto msg_collection_point = std::make_shared<std_msgs::msg::String>();
                auto msg_front_ball = std::make_shared<std_msgs::msg::Bool>();
                auto msg_ball_coordinate = std::make_shared<geometry_msgs::msg::Vector3>();

                std::vector<std::string> class_id;
                std::vector<int> center_x;
                std::vector<int> center_y;

                for (const auto& box : msg->bounding_boxes) {
                    class_id.push_back(box.class_id);

                    int center_x_value = static_cast<int>((box.xmax + box.xmin) / 2);
                    int center_y_value = static_cast<int>((box.ymax + box.ymin) / 2);
                    center_x.push_back(center_x_value);
                    center_y.push_back(center_y_value);
                }

                // center_dist = cv_image_.at<uint32_t>(center_y, center_x);
                // RCLCPP_INFO(this->get_logger(), "%d", center_dist);

                // Vector3d test111 = ct.Rx_Ry_Rz(static_cast<double>(center_x[0]), static_cast<double>(center_y[0]), /*(double)center_dist*/200, pose);
                // cout << "1111111111111111" << endl;
                // Vector3d test111 = ct.Rx_Ry_Rz(center_x, center_y, /*(double)center_dist*/200, pose);

                // msg_ball_coordinate->x = test[0];
                // msg_ball_coordinate->y = test[1];
                // msg_ball_coordinate->z = test[2];
                if(way_point == "c3" || way_point == "c4"){
                    for (size_t i = 0; i < center_x.size(); ++i) {
                        if(way_point == "c3"){
                            if(court_color == "blue"){
                                if(center_y[i] > str_range_y_C3orC5){
                                    if(center_x[i] < str_range_x_C3orC5[0]) msg_collection_point->data = "ST1";
                                    else if(center_x[i] > str_range_x_C3orC5[0] && center_x[i] < str_range_x_C3orC5[1]) msg_collection_point->data = "ST2";
                                    else if(center_x[i] > str_range_x_C3orC5[1] && center_x[i] < str_range_x_C3orC5[2]) msg_collection_point->data = "ST3";
                                    else if(center_x[i] > str_range_x_C3orC5[2]) msg_collection_point->data = "ST4";
                                }
                            }
                            else {
                                if(center_y[i] > str_range_y_C3orC5){
                                    if(center_x[i] < str_range_x_C3orC5[0]) msg_collection_point->data = "ST4";
                                    else if(center_x[i] > str_range_x_C3orC5[0] && center_x[i] < str_range_x_C3orC5[1]) msg_collection_point->data = "ST3";
                                    else if(center_x[i] > str_range_x_C3orC5[1] && center_x[i] < str_range_x_C3orC5[2]) msg_collection_point->data = "ST2";
                                    else if(center_x[i] > str_range_x_C3orC5[2]) msg_collection_point->data = "ST1";
                                }                            
                            }
                        }
                        else if(way_point == "c4"){
                        // RCLCPP_INFO(this->get_logger(), "y@%d,i@%d", center_y[i], i);
                            if(court_color == "blue"){
                                if(center_y[i] > str_range_y_C3orC5){
                                    if(center_x[i] < str_range_x_C3orC5[0]) msg_collection_point->data = "ST8";
                                    else if(center_x[i] > str_range_x_C3orC5[0] && center_x[i] < str_range_x_C3orC5[1]) msg_collection_point->data = "ST7";
                                    else if(center_x[i] > str_range_x_C3orC5[1] && center_x[i] < str_range_x_C3orC5[2]) msg_collection_point->data = "ST6";
                                    else if(center_x[i] > str_range_x_C3orC5[2]) msg_collection_point->data = "ST5";
                                }
                            }
                            else{
                                if(center_y[i] > str_range_y_C3orC5){
                                    if(center_x[i] < str_range_x_C3orC5[0]) msg_collection_point->data = "ST5";
                                    else if(center_x[i] > str_range_x_C3orC5[0] && center_x[i] < str_range_x_C3orC5[1]) msg_collection_point->data = "ST6";
                                    else if(center_x[i] > str_range_x_C3orC5[1] && center_x[i] < str_range_x_C3orC5[2]) msg_collection_point->data = "ST7";
                                    else if(center_x[i] > str_range_x_C3orC5[2]) msg_collection_point->data = "ST8";
                                }
                            }
                        }
                    }
                    way_point = "";
                    _pub_collection_point->publish(*msg_collection_point);
                }

                //ボールが手前かどうか
                if(!way_point.empty()){
                    for (size_t i = 0; i < center_x.size(); ++i) {
                        if(center_y[i] > str_range_y_ST[0]){
                            if(center_x[i] > str_range_x_ST[0] && center_x[i] < str_range_x_ST[1]){
                                if(center_y[i] > str_range_y_ST[1]) msg_front_ball->data = true;
                                else msg_front_ball->data = false;
                                _pub_front_ball->publish(*msg_front_ball);
                            }
                        }
                    }
                    way_point = "";
                }
            
                    // _pub_collection_point->publish(*msg_collection_point);
                    // _pub_ball_coordinate->publish(*msg_ball_coordinate);
                
                // if(msg_collection_point->data != "") _pub_collection_point->publish(*msg_collection_point);
            }
        }

        void DetectionInterface::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &ptr){
            std::shared_ptr<cv_bridge::CvImage> bridge_ = cv_bridge::toCvCopy(ptr, sensor_msgs::image_encodings::TYPE_16UC1);
            cv_image_ = bridge_->image;
        }

        void DetectionInterface::callback_self_pose(const geometry_msgs::msg::Vector3::SharedPtr msg){
            // is_self_pose_range_x_str = (msg->x > str_self_pose_range[0] && msg->x < str_self_pose_range[1]) ? true : false;
            // is_self_pose_range_y_str = (msg->y > str_self_pose_range[2] && msg->y < str_self_pose_range[3]) ? true : false;
            // is_self_pose_range_z_str = (msg->z > str_self_pose_range[4] && msg->z < str_self_pose_range[5]) ? true : false;
            // is_self_pose_range_x_siro = (msg->x > siro_self_pose_range[0] && msg->x < siro_self_pose_range[1]) ? true : false;
            // is_self_pose_range_y_siro = (msg->y > siro_self_pose_range[2] && msg->y < siro_self_pose_range[3]) ? true : false;
            // is_self_pose_range_z_siro = (msg->z > siro_self_pose_range[4] && msg->z < siro_self_pose_range[5]) ? true : false;

            // std::cout << is_self_pose_range_x_str << "@"<< is_self_pose_range_y_str <<"@"<< is_self_pose_range_z_str << std::endl;
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
        bool DetectionInterface::bounday_line(int x, int y, const std::vector<double> point1, const std::vector<double> point2){
            float a = 0;
            float b = 0;
            float Y = 0;
            bool over = false;
            a=(point2[1] - point1[1]) / (point2[0] - point1[0]);
            b=((point2[0] * point1[1]) - (point1[0] * point2[1])) / (point2[0] - point1[0]);
            Y =a*x+b;
            over = (Y >= y) ? true : false;
            return over;
        }

        bool DetectionInterface::rhombus_inside(int x, int y){
            bool top_left = false;
            bool top_right = false;
            bool down_left = false;
            bool down_right = false;

            top_left = bounday_line(x, y, str_range_point1_1_C2, str_range_point1_2_C2);
            top_right = bounday_line(x, y, str_range_point4_1_C2, str_range_point4_2_C2);
            down_left = bounday_line(x, y, str_range_point3_1_C2, str_range_point3_2_C2);
            down_right = bounday_line(x, y, str_range_point2_1_C2, str_range_point2_2_C2);

            // RCLCPP_INFO(this->get_logger(), "tl:%d tr:%d dl:%d dr:%d", top_left, top_right, down_left, down_right);

            return (!top_left && !top_right && down_left && down_right);
        }

}