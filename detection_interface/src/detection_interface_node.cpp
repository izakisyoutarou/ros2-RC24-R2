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

        str_range_y_C3orC5_2(get_parameter("str_range_y_C3orC5_2").as_int()),

        front_suction_check(get_parameter("front_suction_check").as_integer_array()),
        back_suction_check(get_parameter("back_suction_check").as_integer_array()),

        realsense_max_x(get_parameter("realsense_max_x").as_int()),
        realsense_min_x(get_parameter("realsense_min_x").as_int()),
        realsense_min_y(get_parameter("realsense_min_y").as_int()),

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

            //sequncerから、現在のシーケンスを入れて
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

            _sub_realsense_d435i = image_transport::create_subscription(
                this, "/camera/d435i/color/image_raw",
                std::bind(&DetectionInterface::d435iImageCallback, this, std::placeholders::_1),
                "raw"
            );

            _sub_depth_ = image_transport::create_subscription(
                this, "/camera/camera/depth/image_rect_raw",
                std::bind(&DetectionInterface::depthImageCallback, this, std::placeholders::_1),
                "raw"
            );
        
            //sequncerへ
            _pub_collection_point = this->create_publisher<std_msgs::msg::String>("collection_point", _qos);
            _pub_suction_check = this->create_publisher<std_msgs::msg::String>("suction_check", _qos);
            _pub_siro_param = this->create_publisher<detection_interface_msg::msg::SiroParam>("siro_param", _qos);
            _pub_front_ball = this->create_publisher<std_msgs::msg::Bool>("front_ball", _qos);
            _pub_ball_coordinate = this->create_publisher<geometry_msgs::msg::Vector3>("ball_coordinate", _qos);

            //arm_param_caluculatorへ
            _pub_arm_param = this->create_publisher<detection_interface_msg::msg::ArmParam>("arm_param", _qos);

            //yolox_rosへ
            _pub_threshold = this->create_publisher<detection_interface_msg::msg::Threshold>("threshold", _qos);
        }

        void DetectionInterface::callback_c1(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg){
            if(now_sequence == "storage" || now_sequence == "silo"){
                // time_start = chrono::system_clock::now();
                auto msg_collection_point = std::make_shared<std_msgs::msg::String>();
                auto msg_siro_param = std::make_shared<detection_interface_msg::msg::SiroParam>();

                int silo_num = 0;
                int count = 0;
                bool before_ball_place_flag = false;

                //c1カメラから受け取った情報を入れる配列
                std::vector<std::string> class_id;//redballとかblueballとか
                std::vector<int> center_x;//バウンディングボックスの真ん中(x)
                std::vector<int> center_y;//バウンディングボックスの真ん中(y)
                std::vector<int> bbounbox_size;//バウンディングの面積
                std::array<std::array<int, 4>, 5> min_max_xy; //{min_x、max_x、min_y、max_y}
                std::vector<int> ymax;//ボールのバウンディングの右下(y)

                //関数内で使用する配列
                std::vector<int> before_ball_place;//推論情報をとりあえず放り込む
                // std::vector<int> after_ball_place;//領域内に複数のボールが存在した場合の処理が終わった
                std::vector<std::string> ball_color;//ボールの色

                //boxeesの配列からboxの内容を取り出し
                for (const auto& box : msg->bounding_boxes) {

                    if (box.class_id == "silo" && count < 5) {
                        min_max_xy[count][0] = box.xmin;
                        min_max_xy[count][1] = box.xmax;
                        min_max_xy[count][2] = box.ymin;
                        min_max_xy[count][3] = box.ymax;      
                        count++;  // 使用された要素数をインクリメント
                    }

                    if(box.class_id == "redball" || box.class_id == "blueball"){
                        int size = static_cast<int>((box.xmax - box.xmin) * (box.ymax - box.ymin));
                        bbounbox_size.push_back(size);

                        int center_x_value = static_cast<int>((box.xmax + box.xmin) / 2);
                        int center_y_value = static_cast<int>((box.ymax + box.ymin) / 2);
                        center_x.push_back(center_x_value);
                        center_y.push_back(box.ymax);

                        ymax.push_back(box.ymax);

                        class_id.push_back(box.class_id);
                    }
                }

                // for (int z = 0; z < before_ball_place.size(); ++z) {
                //     RCLCPP_INFO(this->get_logger(), "まえ%d@%d,サイズ@%d", z, min_x[z],min_x[z]);
                // }

                // for (int z = 0; z < count; ++z) {
                //     RCLCPP_INFO(this->get_logger(), "Index: %d, xmin: %d, xmax: %d, ymin: %d, ymax: %d",
                //                 z, min_max_xy[z][0], min_max_xy[z][1], min_max_xy[z][2], min_max_xy[z][3]);
                // }

                if(class_id.size() != 0){//何も認識していないときに、下にいったらエラーを出す。そのためのif文
                    if(now_sequence == "storage"){
                        if(way_point == "c2") c1camera_c2(center_x, center_y);
                    }

                    if(now_sequence == "silo"){
                        if(way_point == "c1") c1camera_c1(  ymax, min_max_xy,
                                                            center_x, center_y, bbounbox_size,
                                                            before_ball_place, ball_color, class_id);
                    }
                }

                // time_end = chrono::system_clock::now();
                // RCLCPP_INFO(this->get_logger(), "scan time->%d[ms]", chrono::duration_cast<chrono::milliseconds>(time_end-time_start).count());
            }
        }

        void DetectionInterface::callback_realsense(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg){
            if(now_sequence == "storage" || now_sequence == "collect"){
                auto msg_collection_point = std::make_shared<std_msgs::msg::String>();
                auto msg_front_ball = std::make_shared<std_msgs::msg::Bool>();
                auto msg_ball_coordinate = std::make_shared<geometry_msgs::msg::Vector3>();
                auto msg_threshold = std::make_shared<detection_interface_msg::msg::Threshold>();

                std::vector<std::string> class_id;
                std::vector<int> center_x;
                std::vector<int> center_y;

                std::vector<int> max_x;
                std::vector<int> min_x;
                std::vector<int> min_y;

                for (const auto& box : msg->bounding_boxes) {
                    class_id.push_back(box.class_id);

                    int center_x_value = static_cast<int>((box.xmax + box.xmin) / 2);
                    int center_y_value = static_cast<int>((box.ymax + box.ymin) / 2);
                    center_x.push_back(center_x_value);
                    center_y.push_back(center_y_value);

                    max_x.push_back(box.xmax);
                    min_x.push_back(box.xmin);
                    min_y.push_back(box.ymin);
                }



                // center_dist = cv_image_.at<uint32_t>(center_y, center_x);
                // RCLCPP_INFO(this->get_logger(), "%d", center_dist);

                // Vector3d test111 = ct.Rx_Ry_Rz(static_cast<double>(center_x[0]), static_cast<double>(center_y[0]), /*(double)center_dist*/200, pose);
                // Vector3d test111 = ct.Rx_Ry_Rz(center_x, center_y, /*(double)center_dist*/200, pose);

                // msg_ball_coordinate->x = test[0];
                // msg_ball_coordinate->y = test[1];
                // msg_ball_coordinate->z = test[2];

                if(class_id.size() != 0){//何も認識していないときに、下にいったらエラーを出す。そのためのif文
                    // max_xで閾値以上の値を削除
                    auto new_end_max = std::remove_if(max_x.begin(), max_x.end(), [this](int value) {
                        return value > this->realsense_max_x;
                    });
                    max_x.erase(new_end_max, max_x.end());

                    // min_xで閾値以下の値を削除
                    auto new_end_min = std::remove_if(min_x.begin(), min_x.end(), [this](int value) {
                        return value < this->realsense_min_x;
                    });
                    min_x.erase(new_end_min, min_x.end());
                    
                    auto new_end_ymin = std::remove_if(min_y.begin(), min_y.end(), [this](int value) {
                        return value < this->realsense_min_y;
                    });
                    min_y.erase(new_end_ymin, min_y.end());
                    
                    int xmax_in_max_x = *max_element(begin(max_x), end(max_x));
                    int xmin_in_min_x = *min_element(begin(min_x), end(min_x));
                    int ymin_in_min_y = *min_element(begin(min_y), end(min_y));

                    msg_threshold->xmax = xmax_in_max_x;
                    msg_threshold->xmin = xmin_in_min_x;
                    msg_threshold->ymin = ymin_in_min_y;
                    _pub_threshold->publish(*msg_threshold);

                    // //realseneのc3、c4から見たとき、どこのSTに行くか。ボールが手前か奥か
                    if(way_point == "c3" || way_point == "c6") realsense_c3_c4(xmax_in_max_x, xmin_in_min_x, ymin_in_min_y, class_id, center_x, center_y);
                }
            }
        }

        void DetectionInterface::depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &ptr){
            std::shared_ptr<cv_bridge::CvImage> bridge_ = cv_bridge::toCvCopy(ptr, sensor_msgs::image_encodings::TYPE_16UC1);
            cv_image_ = bridge_->image;
        }

        void DetectionInterface::d435iImageCallback(const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr &ptr){
            auto img_rgb = cv_bridge::toCvCopy(ptr->rgb, "bgr8");
            auto img_depth = cv_bridge::toCvCopy(ptr->depth, sensor_msgs::image_encodings::TYPE_16UC1);

            cv::Mat frame_rgb = img_rgb->image;
            cv::Mat frame_depth = img_depth->image;
            cv::Mat frame_prev;

            bool front_flag;

            //ST系に入ったときにボールの吸着判定
            if(way_point[0] == 'S' && way_point[1] == 'T'){
                auto msg_suction_check = std::make_shared<std_msgs::msg::String>();
                cv::Vec3b pixel_value;
                uint16_t depth_value;
                
                if(msg_front_ball.data){
                    pixel_value = frame_rgb.at<cv::Vec3b>(front_suction_check[0], front_suction_check[1]);
                    depth_value = frame_depth.at<uint16_t>(front_suction_check[0], front_suction_check[1]); 
                    if(depth_value < 300) {
                        uchar blue = pixel_value[0];
                        uchar green = pixel_value[1];
                        uchar red = pixel_value[2];

                        // std::cout << "blue" << (int)blue << std::endl;
                        // std::cout << "green" << (int)green << std::endl;
                        // std::cout << "red" << (int)red << std::endl;

                        if(red > green + 50 && red > blue + 50) msg_suction_check->data = "R"; //赤ボール
                        else if(blue > green + 30 && blue > red + 30) msg_suction_check->data = "B"; //青ボール
                        else if(red > green + 20 && blue > green + 20) msg_suction_check->data = "P"; //紫ボール
                    }
                    else {
                        msg_suction_check->data = ""; //何も吸着できていない
                    }
                }
                else{
                    pixel_value = frame_rgb.at<cv::Vec3b>(back_suction_check[0], back_suction_check[1]);
                    depth_value = frame_depth.at<uint16_t>(front_suction_check[0], front_suction_check[1]); 
                    if(depth_value < 500) {
                        uchar blue = pixel_value[0];
                        uchar green = pixel_value[1];
                        uchar red = pixel_value[2];

                        // std::cout << "blue" << (int)blue << std::endl;
                        // std::cout << "green" << (int)green << std::endl;
                        // std::cout << "red" << (int)red << std::endl;

                        if(red > green + 50 && red > blue + 50) msg_suction_check->data = "R"; //赤ボール
                        else if(blue > green + 30 && blue > red + 30) msg_suction_check->data = "B"; //青ボール
                        else if(red > green + 20 && blue > green + 20) msg_suction_check->data = "P"; //紫ボール
                    }
                    else {
                        msg_suction_check->data = ""; //何も吸着できていない
                    }
                }

                _pub_suction_check->publish(*msg_suction_check);
            }

            ////////////d435iを見る(これは常に見てる)
            // 点の座標を設定
            cv::Point front_point(front_suction_check[1], front_suction_check[0]);
            cv::Point back_point(back_suction_check[1], back_suction_check[0]);

            // 画像に緑色の点を描画（半径2の小さい円として）
            const cv::Scalar greenColor(0, 255, 0);  // BGRで緑色
            cv::circle(frame_rgb, front_point, 2, greenColor, -1);  // 塗りつぶしの円として描画
            cv::circle(frame_rgb, back_point, 2, greenColor, -1);  // 塗りつぶしの円として描画

            cv::Size target_size(1280, 720);
            cv::resize(frame_rgb, frame_prev, target_size);
            cv::imshow("d435i", frame_prev);
            cv::waitKey(1);
            ///////////    
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
            }
        }

        void DetectionInterface::callback_now_sequence(const std_msgs::msg::String::SharedPtr msg){
            now_sequence = msg->data;
        }

        void DetectionInterface::callback_way_point(const std_msgs::msg::String::SharedPtr msg){
            way_point = msg->data;
        }

        void DetectionInterface::c1camera_c2(const std::vector<int> center_x, const std::vector<int> center_y){
            auto msg_collection_point = std::make_shared<std_msgs::msg::String>();

            if(storage_flag){//認識区間内で一回だけc3 or c4 or ""を送るようにする。サイロに向かうところで再度tureになる
                bool is_collection_C4 = true;
                for (size_t i = 0; i < center_x.size(); ++i) {
                    if(rhombus_inside(center_x[i], center_y[i])){//坂上からc1で見たときにひし形の中にある
                        if(court_color == "blue") is_collection_C4 = bounday_line(center_x[i], center_y[i], str_range_point1_blue, str_range_point2_blue);
                        else is_collection_C4 = bounday_line(center_x[i], center_y[i], str_range_point1_red, str_range_point2_red);
                    }
                    if(!is_collection_C4)break;//c3を見つけた瞬間、for文を抜け出す
                }

                if(!is_collection_C4) msg_collection_point->data = "c3";
                else msg_collection_point->data = "c4";

                _pub_collection_point->publish(*msg_collection_point);

                storage_flag = false;
            }
        }

        void DetectionInterface::c1camera_c1(   const std::vector<int> ymax, std::array<std::array<int, 4>, 5>& min_max_xy,
                                                const std::vector<int> center_x, const std::vector<int> center_y, std::vector<int> bbounbox_size,
                                                std::vector<int> before_ball_place, std::vector<std::string> ball_color, const std::vector<std::string> class_id){
            int silo_num = 0;
            
            std::array<std::array<int, 4>, 5> silo_y;//{min_y、min_yとcnterの間、max_yとcenterの間、max_y}

            storage_flag = true;//c1カメラのストレージゾーン認識のflag
            c3_c4_flag = true; //realsenseのc3、c4からの認識

            std::sort(min_max_xy.begin(), min_max_xy.end(), [](const std::array<int, 4>& a, const std::array<int, 4>& b) {
                return a[0] < b[0]; // 1列目で比較
            });

            //サイロのyの閾値
            for(size_t i = 0; i < 5; ++i) {
                int center_line = (min_max_xy[i][2] + min_max_xy[i][3]) / 2;

                silo_y[i][0] = min_max_xy[i][2];
                silo_y[i][1] = (center_line + min_max_xy[i][2]) / 2;
                silo_y[i][2] = (center_line + min_max_xy[i][3]) / 2;
                silo_y[i][3] = min_max_xy[i][3];
            }

            for(size_t i = 0; i < center_x.size(); ++i) {
                silo_num = 0;

                //x座標の判定
                if(court_color == "blue"){
                    if(center_x[i] > min_max_xy[0][0] && center_x[i] < min_max_xy[0][1]) silo_num += 12;
                    else if(center_x[i] > min_max_xy[1][0] && center_x[i] < min_max_xy[1][1]) silo_num += 9;
                    else if(center_x[i] > min_max_xy[2][0] && center_x[i] < min_max_xy[2][1]) silo_num += 6;
                    else if(center_x[i] > min_max_xy[3][0] && center_x[i] < min_max_xy[3][1]) silo_num += 3;
                    else if(center_x[i] > min_max_xy[4][0] && center_x[i] < min_max_xy[4][1]) silo_num += 0;
                }
                else{
                    if(center_x[i] > min_max_xy[0][0] && center_x[i] < min_max_xy[0][1]) silo_num += 0;
                    else if(center_x[i] > min_max_xy[1][0] && center_x[i] < min_max_xy[1][1]) silo_num += 3;
                    else if(center_x[i] > min_max_xy[2][0] && center_x[i] < min_max_xy[2][1]) silo_num += 6;
                    else if(center_x[i] > min_max_xy[3][0] && center_x[i] < min_max_xy[3][1]) silo_num += 9;
                    else if(center_x[i] > min_max_xy[4][0] && center_x[i] < min_max_xy[4][1]) silo_num += 12;
                }

                //ボールの段数
                if(silo_num != 0){
                    int silo_num_count = silo_num;
                    int silo_beside = silo_num / 3;

                    if(center_y[i] > silo_y[silo_beside][0] && center_y[i] < silo_y[silo_beside][1]) silo_num += 1;
                    else if(center_y[i] > silo_y[silo_beside][1] && center_y[i] < silo_y[silo_beside][2]) silo_num += 2;
                    else if(center_y[i] > silo_y[silo_beside][2] && center_y[i] < silo_y[silo_beside][3]) silo_num += 3;

                    // for (int z = 0; z < 5; ++z) {
                    //     RCLCPP_INFO(this->get_logger(), "Index: %d, center_y: %d, min_y: %d, min_y-cen: %d, max_y-cen: %d, max_y: %d",
                    //                 z, center_y[i], silo_y[z][0], silo_y[z][1], silo_y[z][2], silo_y[z][3]);
                    // }

                    // RCLCPP_INFO(this->get_logger(), "まえ%d", silo_num);

                    //yの値を見たときに加算処理が行われたときの判定。行われなかったときは、何もpush_backしない
                    if(silo_num != silo_num_count){
                        before_ball_place.push_back(silo_num);

                        if(class_id[i] == "redball") ball_color.push_back("R");
                        else if(class_id[i] == "blueball") ball_color.push_back("B");

                        c1camera_pick_best_BoundingBox(before_ball_place, bbounbox_size, ball_color);
                    }
                }
            }
        }

        void DetectionInterface::c1camera_pick_best_BoundingBox(std::vector<int> before_ball_place, std::vector<int> bbounbox_size, std::vector<std::string> ball_color){
            for (int i = 0; i < before_ball_place.size(); ++i){
                for (int j = i + 1; j < before_ball_place.size(); ++j){//ex:前のfor文で0番目のとき、次のfor文で1番目が回る(以下ex)
                    if (before_ball_place[i] == before_ball_place[j]) {//0番目と1番目のボールの場所が一緒だったとき
                        if(bbounbox_size[i] > bbounbox_size[j]){//0番目が大きいとき
                            bbounbox_size.erase(bbounbox_size.begin() + j);//バウンディングの面積の1番目を削除
                            before_ball_place.erase(before_ball_place.begin() + j);//ボールの場所の1番目を削除
                            ball_color.erase(ball_color.begin() + j);//ボールの色の1番目を削除
                            --j;//次のループのときに1つ飛ばしてしまうため
                        }
                        else{//1番目が大きいとき
                            bbounbox_size.erase(bbounbox_size.begin() + i);//バウンディングの面積の0番目を削除
                            before_ball_place.erase(before_ball_place.begin() + i);//ボールの場所の0番目を削除
                            ball_color.erase(ball_color.begin() + i);//ボールの色の0番目を削除
                            --i;//次のループのときに1つ飛ばしてしまうため
                            break;//for文の0番目が変更になってしまうため、ループ抜け出し
                        }
                    }
                }
            }

            c1camera_correct_silo_levels(before_ball_place, ball_color);
        }

        void DetectionInterface::c1camera_correct_silo_levels(const std::vector<int> before_ball_place, const std::vector<std::string> ball_color){
            auto msg_siro_param = std::make_shared<detection_interface_msg::msg::SiroParam>();
            std::vector<int> after_ball_place;//領域内に複数のボールが存在した場合の処理が終わった

            for(int i = 0; i < before_ball_place.size(); ++i){
                bool stage_three = false;
                bool stage_two = false;
                bool stage_one = false;
                int ball_height = 0;
                int ball_width = 0;
                int silo_num = 0;

                ball_width = before_ball_place[i] / 3;
                ball_height = before_ball_place[i] % 3;

                if(ball_height != 0){
                    for(int j = 1; j < 3; ++j){//ball_heightの可能性として1%3と2%3と3%3なのでj < 3
                        if(ball_height == j){//その領域にballが他に存在しているか。
                            for(size_t k = 0; k < before_ball_place.size(); ++k){
                                if(before_ball_place[k] == ball_width * 3 + 3) stage_three = true;
                                else if(before_ball_place[k] == ball_width * 3 + 2) stage_two = true;
                                else if(before_ball_place[k] == ball_width * 3 + 1) stage_one = true;
                            }
                            if(stage_three && stage_two && stage_one) silo_num = ball_width * 3 + 1;
                            else if(stage_three && stage_two && !stage_one) silo_num = ball_width * 3 + 2;
                            else if(stage_three && !stage_two && stage_one) silo_num = ball_width * 3 + 2;
                            else if(!stage_three && stage_two && !stage_one) silo_num = ball_width * 3 + 3;
                            else if(!stage_three && !stage_two && stage_one) silo_num = ball_width * 3 + 3;
                            // else if(!stage_three && stage_two && stage_one) silo_num = ball_width * 3 + 3; //考えられない
                        }
                    }
                }
                else{
                    silo_num = ball_width * 3;
                }
                after_ball_place.push_back(silo_num);
            }

            // for (int z = 0; z < after_ball_place.size(); ++z) {
            //     RCLCPP_INFO(this->get_logger(), "まえ%d@%d", z, after_ball_place[z]);
            // }

            for (int i = 0; i < after_ball_place.size(); ++i) {
                msg_siro_param->ball_color[after_ball_place[i] - 1] = ball_color[i];
            }

            _pub_siro_param->publish(*msg_siro_param);
        }

        void DetectionInterface::realsense_c3_c4(int xmax, int xmin, int ymin, const std::vector<std::string> class_id, const std::vector<int> center_x, std::vector<int> center_y){
            auto msg_collection_point = std::make_shared<std_msgs::msg::String>();
            msg_front_ball.data = false;

            int C3orC5_x[5] = { xmin, 
                                ((xmax+xmin)/2 + xmin)/2, 
                                (xmax+xmin)/2, 
                                ((xmax+xmin)/2 + xmax)/2,
                                xmax};

            int C3orC5_y[3] = { ymin,
                                ymin + str_range_y_C3orC5_2,
                                ymin + str_range_y_C3orC5_2*2};

            if(c3_c4_flag){
                for (size_t i = 0; i < center_x.size(); ++i) {
                    if(class_id[i] == "redball" || class_id[i] == "blueball"){
                        if(way_point == "c3"){
                            if(court_color == "blue"){
                                if(center_y[i] < C3orC5_y[2]){
                                    if(center_x[i] > C3orC5_x[0] && center_x[i] < C3orC5_x[1]) msg_collection_point->data = "ST0";
                                    else if(center_x[i] > C3orC5_x[1] && center_x[i] < C3orC5_x[2]) msg_collection_point->data = "ST1";
                                    else if(center_x[i] > C3orC5_x[2] && center_x[i] < C3orC5_x[3]) msg_collection_point->data = "ST2";
                                    else if(center_x[i] > C3orC5_x[3] && center_x[i] < C3orC5_x[4]) msg_collection_point->data = "ST3";
                                }
                            }
                            else {
                                if(center_y[i] < C3orC5_y[2]){
                                    if(center_x[i] > C3orC5_x[0] && center_x[i] < C3orC5_x[1]) msg_collection_point->data = "ST3";
                                    else if(center_x[i] > C3orC5_x[1] && center_x[i] < C3orC5_x[2]) msg_collection_point->data = "ST2";
                                    else if(center_x[i] > C3orC5_x[2] && center_x[i] < C3orC5_x[3]) msg_collection_point->data = "ST1";
                                    else if(center_x[i] > C3orC5_x[3] && center_x[i] < C3orC5_x[4]) msg_collection_point->data = "ST0";
                                }                            
                            }
                        }
                        else if(way_point == "c6"){
                            if(court_color == "blue"){
                                if(center_y[i] < C3orC5_y[2]){
                                    if(center_x[i] > C3orC5_x[0] && center_x[i] < C3orC5_x[1]) msg_collection_point->data = "ST7";
                                    else if(center_x[i] > C3orC5_x[1] && center_x[i] < C3orC5_x[2]) msg_collection_point->data = "ST6";
                                    else if(center_x[i] > C3orC5_x[2] && center_x[i] < C3orC5_x[3]) msg_collection_point->data = "ST5";
                                    else if(center_x[i] > C3orC5_x[3] && center_x[i] < C3orC5_x[4]) msg_collection_point->data = "ST4";
                                }
                            }
                            else{
                                if(center_y[i] < C3orC5_y[2]){
                                    if(center_x[i] > C3orC5_x[0] && center_x[i] < C3orC5_x[1]) msg_collection_point->data = "ST4";
                                    else if(center_x[i] > C3orC5_x[1] && center_x[i] < C3orC5_x[2]) msg_collection_point->data = "ST5";
                                    else if(center_x[i] > C3orC5_x[2] && center_x[i] < C3orC5_x[3]) msg_collection_point->data = "ST6";
                                    else if(center_x[i] > C3orC5_x[3] && center_x[i] < C3orC5_x[4]) msg_collection_point->data = "ST7";
                                }
                            }
                        }

                        if(msg_collection_point->data != "") msg_front_ball.data = realsense_front_ball(center_y[i], C3orC5_y[1]);
                    }
                    if(msg_front_ball.data) break;//手前のボールが見つかった瞬間、for文を抜け出す
                }

                c3_c4_flag = false;

                if(msg_collection_point->data == ""){ 
                    if(way_point == "c3")msg_collection_point->data = "c4";
                    else if(way_point == "c6")msg_collection_point->data = "";//c6の場合は、c3を優先しているにも関わらず行ったので空欄を入れる
                    c3_c4_flag = true;
                }
                way_point = "";

                _pub_collection_point->publish(*msg_collection_point);
                _pub_front_ball->publish(msg_front_ball);
            }
        }

        bool DetectionInterface::realsense_front_ball(int center_y, int threshold_y){
            bool is_front_ball;

            if(center_y < threshold_y) is_front_ball = true;
            else is_front_ball = false;

            return is_front_ball;
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