#include "detection_interface/detection_interface_node.hpp"

namespace detection_interface
{
    DetectionInterface::DetectionInterface(const rclcpp::NodeOptions &options) : DetectionInterface("", options) {}
    DetectionInterface::DetectionInterface(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("detection_interface_node", name_space, options),
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

        str_range_x_C3orC5_2(get_parameter("str_range_x_C3orC5_2").as_int()),
        str_range_y_C3orC5_2(get_parameter("str_range_y_C3orC5_2").as_int()),

        str_range_y_ST2(get_parameter("str_range_y_ST2").as_int()),
        
        siro_ball_range_y(get_parameter("siro_ball_range_y").as_double_array()),
        siro_ball_range_x(get_parameter("siro_ball_range_x").as_double_array()),

        siro_ball_range_y_2(get_parameter("siro_ball_range_y_2").as_int()),
        siro_ball_range_x_2(get_parameter("siro_ball_range_x_2").as_int()),

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
            time_start = chrono::system_clock::now();
            auto msg_collection_point = std::make_shared<std_msgs::msg::String>();
            auto msg_siro_param = std::make_shared<detection_interface_msg::msg::SiroParam>();

            int siro_num = 0;
            bool before_ball_place_flag = false;

            //c1カメラから受けっとった情報を入れる配列
            std::vector<std::string> class_id;//redballとかblueballとか
            std::vector<int> center_x;//バウンディングボックスの真ん中(x)
            std::vector<int> center_y;//バウンディングボックスの真ん中(y)
            std::vector<int> bbounbox_size;//バウンディングの面積
            std::vector<int> min_x;//バウンディングボックスの左上(x)
            std::vector<int> max_y;//バウンディングボックスの右下(y)
            std::vector<int> ymax;

            //関数内で使用する配列
            std::vector<int> before_ball_place;//推論情報をとりあえず放り込む
            std::vector<int> after_ball_place;//領域内に複数のボールが存在した場合の処理が終わった
            std::vector<std::string> ball_color;
            std::vector<std::string> c3_or_c4;

            //boxeesの配列からboxの内容を取り出し
            for (const auto& box : msg->bounding_boxes) {
                class_id.push_back(box.class_id);

                if(box.class_id == "silo"){
                    min_x.push_back(box.xmin);
                    max_y.push_back(box.ymax);
                }

                if(box.class_id == "redball" || box.class_id == "blueball"){
                    int size = static_cast<int>((box.xmax - box.xmin) * (box.ymax - box.ymin));
                    bbounbox_size.push_back(size);

                    int center_x_value = static_cast<int>((box.xmax + box.xmin) / 2);
                    int center_y_value = static_cast<int>((box.ymax + box.ymin) / 2);
                    center_x.push_back(center_x_value);
                    center_y.push_back(center_y_value);

                    ymax.push_back(box.ymax);
                }
            }

            // if(now_sequence == "storage"){
            //     if(way_point == "c2") c1camera_c2(center_x, center_y, c3_or_c4);
            // }

            // if(now_sequence == "silo"){
                // if(way_point == "c1") c1camera_c1(min_x, max_y, class_id, center_x, center_y, before_ball_place, ball_color);
            // }

            // if(before_ball_place.size() != 0){
            //     c1camera_pick_best_BoundingBox(before_ball_place, bbounbox_size, ball_color);
            //     c1camera_correct_silo_levels(before_ball_place, ball_color, after_ball_place)
            // }

            //ひし形モードのときに、ひし形に向かっているとき
            if(now_sequence == "storage"){
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

            if(now_sequence == "silo"){
                if(way_point == "c1"){
                    storage_flag = true;//c1カメラのストレージゾーン認識のflag
                    c3_c4_flag = true; //realsenseのc3、c4からの認識
                    st_flag = true;
                    for (size_t i = 0; i < center_x.size(); ++i) {
                        siro_num = 0;

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
                            // for (int z = 0; z < before_ball_place.size(); ++z) {
                            //     RCLCPP_INFO(this->get_logger(), "まえ%d@%d,サイズ@%d", z, before_ball_place[z],bbounbox_size[z]);
                            // }

                            if(class_id[i] == "redball") ball_color.push_back("R");
                            else if(class_id[i] == "blueball") ball_color.push_back("B");

                            before_ball_place_flag = true;
                        }
                    }
                    if(!before_ball_place_flag) _pub_siro_param->publish(*msg_siro_param);
                }
            }

            //サイロ方面の検出が行われると入るところ
            if(before_ball_place.size() != 0){
                // for (int z = 0; z < before_ball_place.size(); ++z) {
                //     RCLCPP_INFO(this->get_logger(), "まえ%d@%d,サイズ@%d", z, before_ball_place[z],bbounbox_size[z]);
                // }

                //同じ領域にボールが複数存在している場合、バウンディングボックスのサイズが大きい方を採用する。それのところ
                for (int i = 0; i < before_ball_place.size(); ++i){
                    for (int j = i + 1; j < before_ball_place.size(); ++j){
                        if (before_ball_place[i] == before_ball_place[j]) {
                            if(bbounbox_size[i] > bbounbox_size[j]){
                                bbounbox_size.erase(bbounbox_size.begin() + j);
                                before_ball_place.erase(before_ball_place.begin() + j);
                                ball_color.erase(ball_color.begin() + j);
                                --j;
                            }
                            else{
                                bbounbox_size.erase(bbounbox_size.begin() + i);
                                before_ball_place.erase(before_ball_place.begin() + i);
                                ball_color.erase(ball_color.begin() + i);
                                --i;
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
                        for(int j = 0; j < 3; ++j){//ball_heightの可能性として1%3と2%3と3%3なのでd < 3
                            if(ball_height == j){//ballが他に存在しているか。
                                for(size_t k = 0; k < before_ball_place.size(); ++k){
                                    if(before_ball_place[k] == ball_width * 3 + 3) stage_three = true;
                                    if(before_ball_place[k] == ball_width * 3 + 2) stage_two = true;
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

                std::vector<int> min_x;
                std::vector<int> max_y;

                c3_flag = false;

                for (const auto& box : msg->bounding_boxes) {
                    class_id.push_back(box.class_id);

                    int center_x_value = static_cast<int>((box.xmax + box.xmin) / 2);
                    int center_y_value = static_cast<int>((box.ymax + box.ymin) / 2);
                    center_x.push_back(center_x_value);
                    center_y.push_back(center_y_value);

                    min_x.push_back(box.xmin);
                    max_y.push_back(box.ymax);
                }

                // center_dist = cv_image_.at<uint32_t>(center_y, center_x);
                // RCLCPP_INFO(this->get_logger(), "%d", center_dist);

                // Vector3d test111 = ct.Rx_Ry_Rz(static_cast<double>(center_x[0]), static_cast<double>(center_y[0]), /*(double)center_dist*/200, pose);
                // Vector3d test111 = ct.Rx_Ry_Rz(center_x, center_y, /*(double)center_dist*/200, pose);

                // msg_ball_coordinate->x = test[0];
                // msg_ball_coordinate->y = test[1];
                // msg_ball_coordinate->z = test[2];

                // int xmin_in_min_x = min_element(begin(min_x), end(min_x));
                // int ymax_in_max_y = max_element(begin(max_y), end(max_y));

                // //realseneのc3、c4から見たとき、どこのSTに行くか
                // if(way_point == "c3" || way_point == "c4") realsense_c3_c4(xmin_in_min_x, ymax_in_max_y, class_id, center_x, center_y);

                // //realsenseのST系から見たとき、ボールが手前かどうか
                // if(way_point[0] == 'S' && way_point[1] == 'T') realsense_ST(ymax_in_max_y, class_id, center_x, center_y);

                //c3とc4で物体検出をするところ
                if(way_point == "c3" || way_point == "c4"){
                    if(c3_c4_flag){
                    for (size_t i = 0; i < center_x.size(); ++i) {
                        if(way_point == "c3"){
                            if(court_color == "blue"){
                                if(center_y[i] < str_range_y_C3orC5){
                                    if(center_x[i] < str_range_x_C3orC5[0]) msg_collection_point->data = "ST3";
                                    else if(center_x[i] > str_range_x_C3orC5[0] && center_x[i] < str_range_x_C3orC5[1]) msg_collection_point->data = "ST2";
                                    else if(center_x[i] > str_range_x_C3orC5[1] && center_x[i] < str_range_x_C3orC5[2]) msg_collection_point->data = "ST1";
                                    else if(center_x[i] > str_range_x_C3orC5[2]) msg_collection_point->data = "ST0";
                                }
                            }
                            else {
                                if(center_y[i] < str_range_y_C3orC5){
                                    if(center_x[i] < str_range_x_C3orC5[0]) msg_collection_point->data = "ST0";
                                    else if(center_x[i] > str_range_x_C3orC5[0] && center_x[i] < str_range_x_C3orC5[1]) msg_collection_point->data = "ST1";
                                    else if(center_x[i] > str_range_x_C3orC5[1] && center_x[i] < str_range_x_C3orC5[2]) msg_collection_point->data = "ST2";
                                    else if(center_x[i] > str_range_x_C3orC5[2]) msg_collection_point->data = "ST3";
                                }                            
                            }
                            to_c3_flag = true;
                        }
                        else if(way_point == "c4"){
                        // RCLCPP_INFO(this->get_logger(), "y@%d,i@%d", center_y[i], i);
                            if(court_color == "blue"){
                                if(center_y[i] < str_range_y_C3orC5){
                                    if(center_x[i] < str_range_x_C3orC5[0]) msg_collection_point->data = "ST4";
                                    else if(center_x[i] > str_range_x_C3orC5[0] && center_x[i] < str_range_x_C3orC5[1]) msg_collection_point->data = "ST5";
                                    else if(center_x[i] > str_range_x_C3orC5[1] && center_x[i] < str_range_x_C3orC5[2]) msg_collection_point->data = "ST6";
                                    else if(center_x[i] > str_range_x_C3orC5[2]) msg_collection_point->data = "ST7";
                                }
                            }
                            else{
                                if(center_y[i] < str_range_y_C3orC5){
                                    if(center_x[i] < str_range_x_C3orC5[0]) msg_collection_point->data = "ST7";
                                    else if(center_x[i] > str_range_x_C3orC5[0] && center_x[i] < str_range_x_C3orC5[1]) msg_collection_point->data = "ST6";
                                    else if(center_x[i] > str_range_x_C3orC5[1] && center_x[i] < str_range_x_C3orC5[2]) msg_collection_point->data = "ST5";
                                    else if(center_x[i] > str_range_x_C3orC5[2]) msg_collection_point->data = "ST4";
                                }
                            }
                            to_c3_flag = false;
                        }  
                    }

                    c3_c4_flag = false;
                    st_flag = true;

                    way_point = "";

                    _pub_collection_point->publish(*msg_collection_point);
                    }
                }

                //ボールが手前かどうか
                if(way_point[0] == 'S' && way_point[1] == 'T'){
                    if(st_flag){
                    bool msg_flag = false;

                    for (size_t i = 0; i < center_x.size(); ++i) {
                        if(center_y[i] < str_range_y_ST[1]){
                            if(center_x[i] > str_range_x_ST[0] && center_x[i] < str_range_x_ST[1]){
                                if(center_y[i] < str_range_y_ST[0]) msg_front_ball->data = true;
                                else if (center_y[i] > str_range_y_ST[0] && center_y[i] < str_range_y_ST[1])msg_front_ball->data = false;
                                _pub_front_ball->publish(*msg_front_ball);
                                msg_flag = true;
                            }
                        }
                    }

                    if(!msg_flag){
                        RCLCPP_INFO(this->get_logger(), "%d", to_c3_flag);
                        if(to_c3_flag)msg_collection_point->data = "c3";
                        else msg_collection_point->data = "c4";
                        _pub_collection_point->publish(*msg_collection_point);
                        c3_c4_flag = true;
                    }
                    way_point = "";
                    st_flag = false;
                    }
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
            pose[0] = msg->x;
            pose[1] = msg->y;
            pose[2] = msg->z;
        }

        void DetectionInterface::callback_base_control(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
            if(msg->is_restart){
                way_point = "";
                now_sequence = "";
                storage_flag = true;
                c3_c4_flag = true;
                st_flag = true;
                to_c3_flag = true;
            }
        }

        void DetectionInterface::callback_now_sequence(const std_msgs::msg::String::SharedPtr msg){
            now_sequence = msg->data;
        }

        void DetectionInterface::callback_way_point(const std_msgs::msg::String::SharedPtr msg){
            auto msg_siro_param = std::make_shared<detection_interface_msg::msg::SiroParam>();
            way_point = msg->data;

            // if(way_point == "c1"){
            //     if(!c1camera_flag) _pub_siro_param->publish(*msg_siro_param);
            // }
            // std::cout << way_point << std::endl;
        }

        void DetectionInterface::c1camera_c2(const std::vector<int> center_x, const std::vector<int> center_y, std::vector<std::string> c3_or_c4){
            auto msg_collection_point = std::make_shared<std_msgs::msg::String>();

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

                ////////坂上からひし形を見たときに閾値からc3とc4を登録し、その中からc3を先に探索して、c3がなかった場合c4になる
                auto c3 = std::find(c3_or_c4.begin(), c3_or_c4.end(), "c3");
                auto c4 = std::find(c3_or_c4.begin(), c3_or_c4.end(), "c4");

                if(c3 != c3_or_c4.end())msg_collection_point->data = "c3";
                else if(c4 != c3_or_c4.end())msg_collection_point->data = "c4";
                ///////

                _pub_collection_point->publish(*msg_collection_point);

                storage_flag = false;
            }
        }

        void DetectionInterface::c1camera_c1(const std::vector<int> ymax, const std::vector<int> min_x, const std::vector<int> max_y, const std::vector<std::string> class_id, const std::vector<int> center_x, const std::vector<int> center_y, std::vector<int> before_ball_place, std::vector<std::string> ball_color){
            int siro_num = 0;

            storage_flag = true;//c1カメラのストレージゾーン認識のflag
            c3_c4_flag = true; //realsenseのc3、c4からの認識
            st_flag = true;

            int xmin_in_min_x_silo = *min_element(begin(min_x), end(min_x));
            int ymax_in_max_y_silo = *max_element(begin(max_y), end(max_y));

            int xmin_in_min_x_c1 = xmin_in_min_x_silo + siro_ball_range_x_2;
            int xmin_in_min_x_c1_2 = xmin_in_min_x_silo + siro_ball_range_x_2 * 2;
            int xmin_in_min_x_c1_3 = xmin_in_min_x_silo + siro_ball_range_x_2 * 3;
            int xmin_in_min_x_c1_4 = xmin_in_min_x_silo + siro_ball_range_x_2 * 4;

            int ymax_in_max_y_C3orC5 = ymax_in_max_y_silo - siro_ball_range_y_2;
            int ymax_in_max_y_C3orC5_2 = ymax_in_max_y_silo - siro_ball_range_y_2 * 2;
            int ymax_in_max_y_C3orC5_3 = ymax_in_max_y_silo - siro_ball_range_y_2 * 3;

            for (size_t i = 0; i < center_x.size(); ++i) {
                siro_num = 0;

                //ボールの段数
                if(center_y[i] < ymax_in_max_y_C3orC5_3) siro_num += 1;
                else if(center_y[i] > ymax_in_max_y_C3orC5_3 && center_y[i] < ymax_in_max_y_C3orC5_2) siro_num += 2;
                else if(ymax[i] > ymax_in_max_y_C3orC5_2 && ymax[i] < ymax_in_max_y_C3orC5) siro_num += 3;

                //どのサイロか
                if(siro_num != 0){
                    if(court_color == "blue"){
                        if(center_x[i] < xmin_in_min_x_c1) siro_num += 12;
                        else if(center_x[i] > xmin_in_min_x_c1 && center_x[i] < xmin_in_min_x_c1_2) siro_num += 9;
                        else if(center_x[i] > xmin_in_min_x_c1_2 && center_x[i] < xmin_in_min_x_c1_3) siro_num += 6;
                        else if(center_x[i] > xmin_in_min_x_c1_3 && center_x[i] < xmin_in_min_x_c1_4) siro_num += 3;
                        else if(center_x[i] > xmin_in_min_x_c1_4) siro_num += 0;
                    }
                    else{
                        if(center_x[i] < xmin_in_min_x_c1) siro_num += 0;
                        else if(center_x[i] > xmin_in_min_x_c1 && center_x[i] < xmin_in_min_x_c1_2) siro_num += 3;
                        else if(center_x[i] > xmin_in_min_x_c1_2 && center_x[i] < xmin_in_min_x_c1_3) siro_num += 6;
                        else if(center_x[i] > xmin_in_min_x_c1_3 && center_x[i] < xmin_in_min_x_c1_4) siro_num += 9;
                        else if(center_x[i] > xmin_in_min_x_c1_4) siro_num += 12;
                    }
                    before_ball_place.push_back(siro_num);

                    if(class_id[i] == "redball") ball_color.push_back("R");
                    else if(class_id[i] == "blueball") ball_color.push_back("B");
                }
            }
        }

        void DetectionInterface::c1camera_pick_best_BoundingBox(std::vector<int> before_ball_place, std::vector<int> bbounbox_size, std::vector<std::string> ball_color){
            for (int i = 0; i < before_ball_place.size(); ++i){
                for (int j = i + 1; j < before_ball_place.size(); ++j){
                    if (before_ball_place[i] == before_ball_place[j]) {
                        if(bbounbox_size[i] > bbounbox_size[j]){
                            bbounbox_size.erase(bbounbox_size.begin() + j);
                            before_ball_place.erase(before_ball_place.begin() + j);
                            ball_color.erase(ball_color.begin() + j);
                            --j;
                        }
                        else{
                            bbounbox_size.erase(bbounbox_size.begin() + i);
                            before_ball_place.erase(before_ball_place.begin() + i);
                            ball_color.erase(ball_color.begin() + i);
                            --i;
                            break;
                        }
                    }
                }
            }
        }

        void DetectionInterface::c1camera_correct_silo_levels(const std::vector<int> before_ball_place, const std::vector<std::string> ball_color, std::vector<int> after_ball_place){
            auto msg_siro_param = std::make_shared<detection_interface_msg::msg::SiroParam>();

            for(int i = 0; i < before_ball_place.size(); ++i){
                bool stage_three = false;
                bool stage_two = false;
                bool stage_one = false;
                int ball_height = 0;
                int ball_width = 0;
                int siro_num = 0;

                ball_width = before_ball_place[i] / 3;
                ball_height = before_ball_place[i] % 3;

                if(ball_height != 0){
                    for(int j = 0; j < 3; ++j){//ball_heightの可能性として1%3と2%3と3%3なのでd < 3
                        if(ball_height == j){//ballが他に存在しているか。
                            for(size_t k = 0; k < before_ball_place.size(); ++k){
                                if(before_ball_place[k] == ball_width * 3 + 3) stage_three = true;
                                else if(before_ball_place[k] == ball_width * 3 + 2) stage_two = true;
                                else if(before_ball_place[k] == ball_width * 3 + 1) stage_one = true;
                            }
                            if(stage_three && stage_two && stage_one) siro_num = ball_width * 3 + 1;
                            else if(stage_three && stage_two && !stage_one) siro_num = ball_width * 3 + 2;
                            else if(stage_three && !stage_two && stage_one) siro_num = ball_width * 3 + 2;
                            else if(!stage_three && stage_two && !stage_one) siro_num = ball_width * 3 + 3;
                            else if(!stage_three && !stage_two && stage_one) siro_num = ball_width * 3 + 3;
                            // else if(!stage_three && stage_two && stage_one) siro_num = ball_width * 3 + 3; //考えられない
                        }
                    }
                }
                else{
                    siro_num = ball_width * 3;
                }
                after_ball_place.push_back(siro_num);
            }

            for (int i = 0; i < after_ball_place.size(); ++i) {
                    msg_siro_param->ball_color[after_ball_place[i] - 1] = ball_color[i];
            }

            _pub_siro_param->publish(*msg_siro_param);
        }

        void DetectionInterface::realsense_c3_c4(int xmin, int ymax, const std::vector<std::string> class_id, const std::vector<int> center_x, std::vector<int> center_y){
            auto msg_collection_point = std::make_shared<std_msgs::msg::String>();
            int xmin_in_min_x_C3orC5_2 = xmin + str_range_x_C3orC5_2;
            int xmin_in_min_x_C3orC5_3 = xmin + str_range_x_C3orC5_2*2;

            int ymax_in_max_y_C3orC5 = ymax - str_range_y_C3orC5_2;

            if(c3_c4_flag){
                for (size_t i = 0; i < center_x.size(); ++i) {
                    if(class_id[i] == "R" || class_id[i] == "B"){
                        if(way_point == "c3"){
                            if(court_color == "blue"){
                                if(center_y[i] < ymax_in_max_y_C3orC5){
                                    if(center_x[i] < xmin) msg_collection_point->data = "ST0";
                                    else if(center_x[i] > xmin && center_x[i] < xmin_in_min_x_C3orC5_2) msg_collection_point->data = "ST1";
                                    else if(center_x[i] > xmin_in_min_x_C3orC5_2 && center_x[i] < xmin_in_min_x_C3orC5_3) msg_collection_point->data = "ST2";
                                    else if(center_x[i] > xmin_in_min_x_C3orC5_3) msg_collection_point->data = "ST3";
                                }
                            }
                            else {
                                if(center_y[i] < ymax_in_max_y_C3orC5){
                                    if(center_x[i] < xmin) msg_collection_point->data = "ST3";
                                    else if(center_x[i] > xmin && center_x[i] < xmin_in_min_x_C3orC5_2) msg_collection_point->data = "ST2";
                                    else if(center_x[i] > xmin_in_min_x_C3orC5_2 && center_x[i] < xmin_in_min_x_C3orC5_3) msg_collection_point->data = "ST1";
                                    else if(center_x[i] > xmin_in_min_x_C3orC5_3) msg_collection_point->data = "ST0";
                                }                            
                            }
                            to_c3_flag = true;
                        }
                        else if(way_point == "c4"){
                            if(court_color == "blue"){
                                if(center_y[i] < ymax_in_max_y_C3orC5){
                                    if(center_x[i] < xmin) msg_collection_point->data = "ST7";
                                    else if(center_x[i] > xmin && center_x[i] < xmin_in_min_x_C3orC5_2) msg_collection_point->data = "ST6";
                                    else if(center_x[i] > xmin_in_min_x_C3orC5_2 && center_x[i] < xmin_in_min_x_C3orC5_3) msg_collection_point->data = "ST5";
                                    else if(center_x[i] > xmin_in_min_x_C3orC5_3) msg_collection_point->data = "ST4";
                                }
                            }
                            else{
                                if(center_y[i] < ymax_in_max_y_C3orC5){
                                    if(center_x[i] < xmin) msg_collection_point->data = "ST4";
                                    else if(center_x[i] > xmin && center_x[i] < xmin_in_min_x_C3orC5_2) msg_collection_point->data = "ST5";
                                    else if(center_x[i] > xmin_in_min_x_C3orC5_2 && center_x[i] < xmin_in_min_x_C3orC5_3) msg_collection_point->data = "ST6";
                                    else if(center_x[i] > xmin_in_min_x_C3orC5_3) msg_collection_point->data = "ST7";
                                }
                            }
                        }
                        to_c3_flag = false;
                    }
                }

                c3_c4_flag = false;

                if(msg_collection_point->data == ""){ 
                    msg_collection_point->data = way_point;
                    c3_c4_flag = true;
                }
                way_point = "";

                _pub_collection_point->publish(*msg_collection_point);
            }
        }

        void DetectionInterface::realsense_ST(int ymax, const std::vector<std::string> class_id, const std::vector<int> center_x, std::vector<int> center_y){
            auto msg_collection_point = std::make_shared<std_msgs::msg::String>();
            auto msg_front_ball = std::make_shared<std_msgs::msg::Bool>();

            if(st_flag){
                bool msg_flag = false;
                int ymax_in_max_y_ST2 = ymax - str_range_y_ST2;

                for (size_t i = 0; i < center_x.size(); ++i) {
                    if(class_id[i] == "R" || class_id[i] == "B"){
                        if(center_y[i] < ymax){
                            if(center_x[i] > str_range_x_ST[0] && center_x[i] < str_range_x_ST[1]){
                                if(center_y[i] < ymax_in_max_y_ST2) msg_front_ball->data = true;
                                else msg_front_ball->data = false;

                                _pub_front_ball->publish(*msg_front_ball);
                                msg_flag = true;
                            }
                        }
                    }
                }

                if(!msg_flag){
                    if(to_c3_flag)msg_collection_point->data = "c3";
                    else msg_collection_point->data = "c4";

                    _pub_collection_point->publish(*msg_collection_point);
                    c3_c4_flag = true;
                }
                way_point = "";
                st_flag = false;
            }
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