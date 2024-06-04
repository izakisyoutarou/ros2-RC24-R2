#include "detection_interface/detection_interface_node.hpp"

namespace detection_interface
{
    DetectionInterface::DetectionInterface(const rclcpp::NodeOptions &options) : DetectionInterface("", options) {}
    DetectionInterface::DetectionInterface(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("detection_interface_node", name_space, options),

        front_suction_check_point(get_parameter("front_suction_check_point").as_integer_array()),
        back_suction_check_point(get_parameter("back_suction_check_point").as_integer_array()),
        front_depth_suction_check_value(get_parameter("front_depth_suction_check_value").as_int()),
        back_depth_suction_check_value(get_parameter("back_depth_suction_check_value").as_int()),

        realsense_max_x(get_parameter("realsense_max_x").as_int()),
        realsense_min_x(get_parameter("realsense_min_x").as_int()),
        realsense_max_y(get_parameter("realsense_max_y").as_int()),

        c2node_threshold_x(get_parameter("c2node_threshold_x").as_int()),

        str_range_y_C3orC5(get_parameter("str_range_y_C3orC5").as_int()),
        str_range_x_C3orC5(get_parameter("str_range_x_C3orC5").as_int()),

        court_color(get_parameter("court_color").as_string())
        {
            //yolox_ros_cppのc1から
            _sub_c1 = this->create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>(
                "yolox/c1",
                _qos,
                std::bind(&DetectionInterface::callback_c1camera, this, std::placeholders::_1)
            );

            //yolox_ros_cppのrealsense_d455から
            _sub_realsense_d455 = this->create_subscription<bboxes_ex_msgs::msg::BoundingBoxes>(
                "yolox/realsense_d455",
                _qos,
                std::bind(&DetectionInterface::callback_realsense_d455, this, std::placeholders::_1)
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

            _sub_realsense_d435i = this->create_subscription<realsense2_camera_msgs::msg::RGBD>(
                "/camera/d435i/rgbd",
                _qos,
                std::bind(&DetectionInterface::d435iImageCallback, this, std::placeholders::_1)
            );
        
            //sequncerへ
            _pub_collection_point = this->create_publisher<std_msgs::msg::String>("collection_point", _qos);
            _pub_suction_check = this->create_publisher<std_msgs::msg::String>("suction_check", _qos);
            _pub_silo_param = this->create_publisher<detection_interface_msg::msg::SiloParam>("silo_param", _qos);
            _pub_ball_coordinate = this->create_publisher<geometry_msgs::msg::Vector3>("ball_coordinate", _qos);
        }

        void DetectionInterface::callback_c1camera(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg){
            if(now_sequence == "silo" && way_point == "c1" || now_sequence == "collect" && way_point == "c2"){
                // time_start = chrono::system_clock::now();

                int silo_count = 0;

                bool c1camera_count_flag = false;
                bool c1camera_ball_flag = false;
                
                //c1カメラから受け取った情報を入れる配列
                std::vector<std::string> class_id_c1camera;//redballとかblueballとか
                std::vector<int> center_x_c1camera;//バウンディングボックスの真ん中(x)
                std::vector<int> center_y_c1camera;//バウンディングボックスの真ん中(y)
                std::vector<int> bbounbox_size_c1camera;//バウンディングの面積
                std::vector<int> ymax_c1camera;//ボールのバウンディングの右下(y)

                std::array<std::array<int, 4>, 5> min_max_xy; //{min_x、max_x、min_y、max_y}

                //boxeesの配列からboxの内容を取り出し
                for (const auto& box : msg->bounding_boxes) {

                    if (box.class_id == "silo" && silo_count < 5) {
                        min_max_xy[silo_count][0] = box.xmin;
                        min_max_xy[silo_count][1] = box.xmax;
                        min_max_xy[silo_count][2] = box.ymin;
                        min_max_xy[silo_count][3] = box.ymax;
                        silo_count++;// 使用された要素数をインクリメント
                    }

                    if(box.class_id == "redball" || box.class_id == "blueball"){
                        int center_x_value = static_cast<int>((box.xmax + box.xmin) / 2);
                        int center_y_value = static_cast<int>((box.ymax + box.ymin) / 2);
                        center_x_c1camera.push_back(center_x_value);
                        center_y_c1camera.push_back(center_y_value);

                        class_id_c1camera.push_back(box.class_id);

                        int size = static_cast<int>((box.xmax - box.xmin) * (box.ymax - box.ymin));
                        bbounbox_size_c1camera.push_back(size);

                        ymax_c1camera.push_back(box.ymax);
                    }
                }
                
                // for (int z = 0; z < center_x.size(); ++z) {
                //     RCLCPP_INFO(this->get_logger(), "まえ%d@%d", z, center_x[z]);
                // }

                // for (int z = 0; z < count; ++z) {
                //     RCLCPP_INFO(this->get_logger(), "Index: %d, xmin: %d, xmax: %d, ymin: %d, ymax: %d",
                //                 z, min_max_xy[z][0], min_max_xy[z][1], min_max_xy[z][2], min_max_xy[z][3]);
                // }

                if(silo_count == 5){//サイロが5個検出できたとき
                    if(now_sequence == "silo"){
                        if(way_point == "c1")c1camera_c1node(ymax_c1camera, min_max_xy, center_x_c1camera, center_y_c1camera, bbounbox_size_c1camera, class_id_c1camera);
                    }
                }
                
                if(center_x_c1camera.size() != 0){
                    if(now_sequence == "collect"){
                        if(way_point == "c2")c1camera_c2node(ymax_c1camera, center_x_c1camera);
                    }
                }

                // time_end = chrono::system_clock::now();
                // RCLCPP_INFO(this->get_logger(), "scan time->%d[ms]", chrono::duration_cast<chrono::microseconds>(time_end-time_start).count());
            }
        }

        void DetectionInterface::c1camera_c1node(   const std::vector<int> ymax, std::array<std::array<int, 4>, 5>& min_max_xy,
                                                const std::vector<int> center_x, const std::vector<int> center_y, std::vector<int> bbounbox_size,
                                                const std::vector<std::string> class_id){
            if(silo_flag){
                int silo_num = 0;
                bool is_ball_color_push_back = false;
                auto msg_silo_param = std::make_shared<detection_interface_msg::msg::SiloParam>();

                silo_flag = false;
                
                std::array<std::array<int, 4>, 5> silo_y;//{min_y、min_yとcnterの間、max_yとcenterの間、max_y}
                std::vector<int> before_ball_place;//推論情報をとりあえず放り込む
                std::vector<std::string> ball_color;//ボールの色

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
                    if(center_x[i] > min_max_xy[0][0] && center_x[i] < min_max_xy[0][1]) silo_num += 0;
                    else if(center_x[i] > min_max_xy[1][0] && center_x[i] < min_max_xy[1][1]) silo_num += 3;
                    else if(center_x[i] > min_max_xy[2][0] && center_x[i] < min_max_xy[2][1]) silo_num += 6;
                    else if(center_x[i] > min_max_xy[3][0] && center_x[i] < min_max_xy[3][1]) silo_num += 9;
                    else if(center_x[i] > min_max_xy[4][0] && center_x[i] < min_max_xy[4][1]) silo_num += 12;

                    //y座標の判定
                    if(silo_num == 0 || silo_num == 3 || silo_num == 6 ||silo_num == 9 ||silo_num == 12){
                        int silo_num_count = silo_num;
                        int silo_beside = silo_num / 3;

                        if(ymax[i] > silo_y[silo_beside][0] && ymax[i] <= silo_y[silo_beside][1]) silo_num += 1;
                        else if(ymax[i] > silo_y[silo_beside][1] && ymax[i] <= silo_y[silo_beside][2]) silo_num += 2;
                        else if(ymax[i] > silo_y[silo_beside][2] && ymax[i] <= silo_y[silo_beside][3]) silo_num += 3;

                        // for (int z = 0; z < 5; ++z) {
                        //     RCLCPP_INFO(this->get_logger(), "Index: %d, ymax: %d, min_y: %d, min_y-cen: %d, max_y-cen: %d, max_y: %d",
                        //                 z, ymax[i], silo_y[z][0], silo_y[z][1], silo_y[z][2], silo_y[z][3]);
                        // }

                        //yの値を見たときに加算処理が行われたときの判定。行われなかったときは、何もpush_backしない
                        if(silo_num != silo_num_count){
                            before_ball_place.push_back(silo_num);

                            if(class_id[i] == "redball") ball_color.push_back("R");
                            else if(class_id[i] == "blueball") ball_color.push_back("B");

                            is_ball_color_push_back = true;
                        }
                    }
                }

                // for (int z = 0; z < before_ball_place.size(); ++z) {
                //     RCLCPP_INFO(this->get_logger(), "まえ%dbefore%d", z, before_ball_place[z], ball_color[z]);
                //     std::cout << ball_color[z] << std::endl;
                // }

                //何もpush_backしなかった場合は、次のステップに遷移しない
                if(is_ball_color_push_back) c1camera_pick_best_BoundingBox(before_ball_place, bbounbox_size, ball_color);
                else{
                    _pub_silo_param->publish(*msg_silo_param);//何もサイロに入っていないときにすべてが空欄のsilo_paramaをpubする
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
            auto msg_silo_param = std::make_shared<detection_interface_msg::msg::SiloParam>();
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

                //ex:1番下がないときに下から2番目にあると空中にある判定になり、判定を1番下にするところ
                if(ball_height != 0){
                    for(int j = 1; j < 3; ++j){//ball_heightの可能性として1%3と2%3と3%3なのでj < 3
                        if(ball_height == j){
                            for(size_t k = 0; k < before_ball_place.size(); ++k){
                                if(before_ball_place[k] == ball_width * 3 + 3) stage_three = true;
                                else if(before_ball_place[k] == ball_width * 3 + 2) stage_two = true;
                                else if(before_ball_place[k] == ball_width * 3 + 1) stage_one = true;
                            }
                            if(stage_three && stage_two && stage_one) silo_num = before_ball_place[i];//そのサイロはすべて埋まっているから
                            else if(stage_three && stage_two && !stage_one) silo_num = ball_width * 3 + 2;
                            else if(stage_three && !stage_two && stage_one) silo_num = ball_width * 3 + 2;
                            else if(!stage_three && stage_two && !stage_one) silo_num = ball_width * 3 + 3;
                            else if(!stage_three && !stage_two && stage_one) silo_num = ball_width * 3 + 3;
                            else if(!stage_three && stage_two && stage_one) silo_num = before_ball_place[i];//状況的にはありえないが、推論結果でなってしまう場合がある
                                                                                                            //このパターンは実質的にボールを3個入っているので、シーケンサ側で優先度が最低。
                        }
                    }
                }
                else{
                    silo_num = ball_width * 3;//このパターンに入るのは、before_ball_placeで3,6,9,12,15の一番下
                }
                // RCLCPP_INFO(this->get_logger(), "silo_num%d", silo_num);
                after_ball_place.push_back(silo_num);
            }

            // RCLCPP_INFO(this->get_logger(), "after%d", after_ball_place.size());

            for (int i = 0; i < after_ball_place.size(); ++i) {
                //after_ball_placeにpush_backされるのは1~15なので、そのままでは配列のindex指定に使えない。なので-1している
                msg_silo_param->ball_color[after_ball_place[i] - 1] = ball_color[i];
            }

            // for (int z = 0; z < 15; ++z) {
            //     // RCLCPP_INFO(this->get_logger(), "まえ%d@%d", z, center_x[z]);
            //     std::cout << z << ":" << msg_silo_param->ball_color[z] << std::endl;
            // }

            _pub_silo_param->publish(*msg_silo_param);
        }

        void DetectionInterface::c1camera_c2node(const std::vector<int> ymax, const std::vector<int> center_x){
            if(c1caera_c2ode_flag){
                auto msg_collection_point = std::make_shared<std_msgs::msg::String>();

                c1caera_c2ode_flag = false;

                auto max_it = std::max_element(ymax.begin(), ymax.end());
                int max_index = std::distance(ymax.begin(), max_it);

                if(court_color == "blue"){
                    if(center_x[max_index] > c2node_threshold_x) msg_collection_point->data = "c8";
                    else msg_collection_point->data = "c7";
                }
                else if(court_color == "red"){
                    if(center_x[max_index] > c2node_threshold_x) msg_collection_point->data = "c7";
                    else msg_collection_point->data = "c8";
                }

                _pub_collection_point->publish(*msg_collection_point);
            }
        }

        void DetectionInterface::callback_realsense_d455(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg){
            if(now_sequence == "storage"){
                if(way_point == "c3_a" || way_point == "c3_b" || way_point == "c6_a" || way_point == "c6_b"){
                    bool realsense_count_flag = false;
                    auto msg_collection_point = std::make_shared<std_msgs::msg::String>();

                    //realsenseから受け取った情報を入れる配列
                    std::vector<int> center_x_realsense;//バウンディングボックスの真ん中(x)
                    std::vector<int> center_y_realsense;//バウンディングボックスの真ん中(y)
                    std::vector<int> center_depth_realsense;//バウンディングボックスの真ん中のdepth(y)
                    std::vector<int> rbp_ymax_realsense;//ボールのバウンディングの右下(y)(赤、青、紫)
                    std::vector<int> rbp_xmax_realsense;//ボールのバウンディングの右下(x)(赤、青、紫)
                    std::vector<int> rbp_xmin_realsense;//ボールのバウンディングの左上(x)(赤、青、紫)
                    std::vector<int> rb_ymax_realsense;//ボールのバウンディングの右下(y)(赤、青)
                    std::vector<int> rb_xmax_realsense;//ボールのバウンディングの右下(y)(赤、青)

                    silo_flag = true;

                    for (const auto& box : msg->bounding_boxes) {

                        rbp_ymax_realsense.push_back(box.ymax);
                        rbp_xmax_realsense.push_back(box.xmax);
                        rbp_xmin_realsense.push_back(box.xmin);

                        if(box.class_id == "redball" || box.class_id == "blueball"){
                            int center_x_value = static_cast<int>((box.xmax + box.xmin) / 2);
                            int center_y_value = static_cast<int>((box.ymax + box.ymin) / 2);

                            center_x_realsense.push_back(center_x_value);
                            center_y_realsense.push_back(center_y_value);
                            center_depth_realsense.push_back(box.center_dist);
                            rb_ymax_realsense.push_back(box.ymax);
                            rb_xmax_realsense.push_back(box.xmax);
                        }
                    }

                    // for (int z = 0; z < center_depth.size(); ++z) {
                    //     RCLCPP_INFO(this->get_logger(), "まえ%d@%d", z, center_depth[z]);
                    // }

                    // for (int z = 0; z < center_depth_realsense.size(); ++z) {
                    //     RCLCPP_INFO(this->get_logger(), "まえ%d@%d", z, center_depth_realsense[z]);
                    // }

                    if(rbp_ymax_realsense.size() != 0){//何も認識していないときに、下にいったらエラーを出す。そのためのif文
                        //realseneのc3、c4から見たとき、どこのSTに行くか。ボールが手前か奥か
                        auto rbp_x_max = std::remove_if(rbp_xmax_realsense.begin(), rbp_xmax_realsense.end(), [this](int value){
                            return value > this->realsense_max_x;
                        });
                        rbp_xmax_realsense.erase(rbp_x_max, rbp_xmax_realsense.end());

                        auto rbp_x_min = std::remove_if(rbp_xmin_realsense.begin(), rbp_xmin_realsense.end(), [this](int value){
                            return value < this->realsense_min_x;
                        });
                        rbp_xmin_realsense.erase(rbp_x_min, rbp_xmin_realsense.end());

                        auto rbp_y_max = std::remove_if(rbp_ymax_realsense.begin(), rbp_ymax_realsense.end(), [this](int value){
                            return value > this->realsense_max_y;
                        });
                        rbp_ymax_realsense.erase(rbp_y_max, rbp_ymax_realsense.end());

                        int rbp_ymax = *max_element(begin(rbp_ymax_realsense), end(rbp_ymax_realsense));
                        int rbp_xmax = *max_element(begin(rbp_xmax_realsense), end(rbp_xmax_realsense));
                        int rbp_xmin = *min_element(begin(rbp_xmin_realsense), end(rbp_xmin_realsense));

                        // RCLCPP_INFO(this->get_logger(), "ymax%d,xmax%d,xmin%d" ,rbp_ymax,rbp_xmax,rbp_xmin);

                        realsense_c3_c6node(center_x_realsense, center_y_realsense, center_depth_realsense, rb_ymax_realsense, rb_xmax_realsense, rbp_ymax, rbp_xmax, rbp_xmin);
                    }
                    else {
                        if(way_point == "c6_b") msg_collection_point->data = "c6_a";
                        else if(way_point == "c6_a") msg_collection_point->data = "c3_a";
                        else if(way_point == "c3_a") msg_collection_point->data = "c3_b";
                        else if(way_point == "c3_b") msg_collection_point->data = "";
                        _pub_collection_point->publish(*msg_collection_point);
                    }
                }
            }       
        }

        void DetectionInterface::realsense_c3_c6node(   const std::vector<int> center_x, const std::vector<int> center_y, const std::vector<int> center_depth, 
                                                        const std::vector<int> rb_ymax, const std::vector<int> rb_xmax, const int rbp_ymax, const int rbp_xmax, const int rbp_xmin){
            if(c3_c4_flag){
                auto msg_ball_coordinate = std::make_shared<geometry_msgs::msg::Vector3>();
                auto msg_collection_point = std::make_shared<std_msgs::msg::String>();

                threshold_count++;

                c1caera_c2ode_flag = true;
                c3_c4_flag = false;

                
                if(way_point == "c6_b"  && threshold_count == 1){
                    if(court_color == "blue"){
                        threshold_ymax = rbp_ymax - str_range_y_C3orC5 * 2;
                        threshold_xmin = rbp_xmin + str_range_x_C3orC5 * 2;
                    }
                    if(court_color == "red"){
                        threshold_ymax = rbp_ymax - str_range_y_C3orC5 * 2;
                        threshold_xmax = rbp_xmax + str_range_x_C3orC5 * 2;
                    }
                }

                // int threshold_ymax = rbp_ymax - str_range_y_C3orC5 * 2;//一番手前からボール2個分引いている
                int threshold_xmax = rbp_xmax - str_range_x_C3orC5 * 2;//一番右からボール2個分引いている
                // int threshold_xmin = rbp_xmin + str_range_x_C3orC5 * 2;//一番左からボール2個分足している

                if(court_color == "blue"){
                    threshold_xmax = rbp_xmax - str_range_x_C3orC5 * 2;
                }
                else if(court_color == "red"){
                    threshold_xmin = rbp_xmin + str_range_x_C3orC5 * 2;
                }

                // RCLCPP_INFO(this->get_logger(), "ymax%d,xmax%d,xmin%d" ,threshold_ymax,threshold_xmax,threshold_xmin);

                std::vector<std::vector<int>> xy_realsense_list;

                if(center_x.size() != 0){
                    for(size_t i = 0; i < center_x.size(); i++){
                        if(court_color == "blue"){
                            if(way_point == "c3_a" || way_point == "c6_a"){
                                if(rb_ymax[i] > threshold_ymax && rb_xmax[i] > threshold_xmax){//ひし形の中で微妙な位置にいるボールに対処するためにxmaxを取得
                                    xy_realsense_list.push_back({center_x[i], center_y[i], center_depth[i]});
                                    // RCLCPP_INFO(this->get_logger(), "depth%d" ,center_depth[i]);
                                }
                            }
                            else if(way_point == "c3_b" || way_point == "c6_b"){
                                if(rb_ymax[i] > threshold_ymax && center_x[i] < threshold_xmin){
                                    xy_realsense_list.push_back({center_x[i], center_y[i], center_depth[i]});
                                    // RCLCPP_INFO(this->get_logger(), "depth%d" ,center_depth[i]);
                                }
                            }
                        }
                        else if(court_color == "red"){
                            if(way_point == "c3_a" || way_point == "c6_a"){
                                if(rb_ymax[i] > threshold_ymax && center_x[i] < threshold_xmin){//ひし形の中で微妙な位置にいるボールに対処するためにxmaxを取得
                                    xy_realsense_list.push_back({center_x[i], center_y[i], center_depth[i]});
                                    // RCLCPP_INFO(this->get_logger(), "depth%d" ,center_depth[i]);
                                }
                            }
                            else if(way_point == "c3_b" || way_point == "c6_b"){
                                if(rb_ymax[i] > threshold_ymax && center_x[i] > threshold_xmax){
                                    xy_realsense_list.push_back({center_x[i], center_y[i], center_depth[i]});
                                    // RCLCPP_INFO(this->get_logger(), "depth%d" ,center_depth[i]);
                                }
                            }
                        }
                    }
                    if(xy_realsense_list.size() != 0){
                        int target_value = 640;//realsenseのスクリーンの真ん中
                        int closest_index = 0;

                        int closest_distance = std::numeric_limits<int>::max();

                        for (size_t i = 0; i < xy_realsense_list.size(); ++i) {
                            int distance = std::abs(xy_realsense_list[i][0] - target_value);
                            if (distance < closest_distance) {
                                closest_distance = distance;
                                closest_index = i;
                            }
                        }

                        Vector3d test = ct.Rx_Ry_Rz(static_cast<double>(xy_realsense_list[closest_index][0]), static_cast<double>(xy_realsense_list[closest_index][1]), (double)xy_realsense_list[closest_index][2], pose);

                        msg_ball_coordinate->x = test[0];
                        msg_ball_coordinate->y = test[1];
                        msg_ball_coordinate->z = test[2];

                        _pub_ball_coordinate->publish(*msg_ball_coordinate);
                    }
                    else{
                        if(way_point == "c6_b") msg_collection_point->data = "c6_a";
                        else if(way_point == "c6_a") msg_collection_point->data = "c3_a";
                        else if(way_point == "c3_a") msg_collection_point->data = "c3_b";
                        else if(way_point == "c3_b") msg_collection_point->data = "";
                        _pub_collection_point->publish(*msg_collection_point);
                    }
                }
                else{
                    if(way_point == "c6_b") msg_collection_point->data = "c6_a";
                    else if(way_point == "c6_a") msg_collection_point->data = "c3_a";
                    else if(way_point == "c3_a") msg_collection_point->data = "c3_b";
                    else if(way_point == "c3_b") msg_collection_point->data = "";
                    _pub_collection_point->publish(*msg_collection_point);
                }
            }
        }

        void DetectionInterface::d435iImageCallback(const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr &ptr){
            if(now_sequence == "storage"){
                if(way_point == "c3_a" || way_point == "c3_b" || way_point == "c6_a" || way_point == "c6_b" || way_point == "coord"){
                    auto img_rgb = cv_bridge::toCvCopy(ptr->rgb, "bgr8");
                    auto img_depth = cv_bridge::toCvCopy(ptr->depth, sensor_msgs::image_encodings::TYPE_16UC1);

                    cv::Mat frame_rgb = img_rgb->image;
                    cv::Mat frame_depth = img_depth->image;
                    cv::Mat frame_prev;

                    cv::Mat frame_hsv;
                    cv::cvtColor(frame_rgb, frame_hsv, cv::COLOR_BGR2HSV);

                    //ST系に入ったときにボールの吸着判定
                    auto msg_suction_check = std::make_shared<std_msgs::msg::String>();

                    int rgb_suction_check_point_y = 0;
                    int rgb_suction_check_point_x = 0;

                    bool suction_check_flag = false;

                    std::array<int, 9> blue_values, green_values, red_values;
                    cv::Vec3b median_color;

                    /////////////////////////////////////depthの中央値を計算
                    std::vector<uint16_t> front_center_dist = {   
                        frame_depth.at<uint16_t>(front_suction_check_point[0], front_suction_check_point[1]), 
                        frame_depth.at<uint16_t>(front_suction_check_point[0]-1, front_suction_check_point[1]), 
                        frame_depth.at<uint16_t>(front_suction_check_point[0]-1, front_suction_check_point[1]+1), 
                        frame_depth.at<uint16_t>(front_suction_check_point[0], front_suction_check_point[1]+1),
                        frame_depth.at<uint16_t>(front_suction_check_point[0]+1, front_suction_check_point[1]+1), 
                        frame_depth.at<uint16_t>(front_suction_check_point[0]+1, front_suction_check_point[1]),
                        frame_depth.at<uint16_t>(front_suction_check_point[0]+1, front_suction_check_point[1]-1), 
                        frame_depth.at<uint16_t>(front_suction_check_point[0], front_suction_check_point[1]-1),
                        frame_depth.at<uint16_t>(front_suction_check_point[0]-1, front_suction_check_point[1]-1)
                    };

                    std::vector<uint16_t> back_center_dist = {   
                        frame_depth.at<uint16_t>(back_suction_check_point[0], back_suction_check_point[1]), 
                        frame_depth.at<uint16_t>(back_suction_check_point[0]-1, back_suction_check_point[1]), 
                        frame_depth.at<uint16_t>(back_suction_check_point[0]-1, back_suction_check_point[1]+1), 
                        frame_depth.at<uint16_t>(back_suction_check_point[0], back_suction_check_point[1]+1),
                        frame_depth.at<uint16_t>(back_suction_check_point[0]+1, back_suction_check_point[1]+1), 
                        frame_depth.at<uint16_t>(back_suction_check_point[0]+1, back_suction_check_point[1]),
                        frame_depth.at<uint16_t>(back_suction_check_point[0]+1, back_suction_check_point[1]-1), 
                        frame_depth.at<uint16_t>(back_suction_check_point[0], back_suction_check_point[1]-1),
                        frame_depth.at<uint16_t>(back_suction_check_point[0]-1, back_suction_check_point[1]-1)
                    };

                    std::sort(front_center_dist.begin(), front_center_dist.end());
                    std::sort(back_center_dist.begin(), back_center_dist.end());

                    uint16_t front_depth_average = front_center_dist[front_center_dist.size() / 2];
                    uint16_t back_depth_average = back_center_dist[back_center_dist.size() / 2];

                    // RCLCPP_INFO(this->get_logger(), "front_depth%d" , front_depth_average);
                    // RCLCPP_INFO(this->get_logger(), "back_depth%d" , back_depth_average);

                    if(back_depth_average > 0 && back_depth_average > front_depth_suction_check_value && back_depth_average < back_depth_suction_check_value){
                    // if(back_depth_average != 0){
                        rgb_suction_check_point_y = back_suction_check_point[0];
                        rgb_suction_check_point_x = back_suction_check_point[1];
                        suction_check_flag = true;
                    }
                    else if(front_depth_average < front_depth_suction_check_value){
                        rgb_suction_check_point_y = front_suction_check_point[0];
                        rgb_suction_check_point_x = front_suction_check_point[1];
                        suction_check_flag = true;
                    }
                    else{
                        msg_suction_check->data = ""; //何も吸着できていない
                    }

                    // if(front_depth_average < front_depth_suction_check_value){
                    //     rgb_suction_check_point_y = front_suction_check_point[0];
                    //     rgb_suction_check_point_x = front_suction_check_point[1];
                    //     suction_check_flag = true;
                    // }
                    // else if(0 < back_depth_average && back_depth_average > front_depth_suction_check_value && back_depth_average < back_depth_suction_check_value){
                    //     rgb_suction_check_point_y = back_suction_check_point[0];
                    //     rgb_suction_check_point_x = back_suction_check_point[1];
                    //     suction_check_flag = true;
                    // }
                    // else{
                    //     msg_suction_check->data = ""; //何も吸着できていない
                    // }
                    
                    if(suction_check_flag){
                        std::vector<cv::Point> offsets = {
                            {0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {0, -1}, {-1, 0}
                        };

                        int count = 0;
                        double hue = 0.0;
                        double saturation = 0.0;
                        double value = 0.0;

                        for(const auto& offset : offsets) {
                            int px = rgb_suction_check_point_x + offset.x;
                            int py = rgb_suction_check_point_y + offset.y;
                            cv::Vec3b pixel = frame_hsv.at<cv::Vec3b>(py, px);

                            hue += pixel[0];
                            saturation += pixel[1];
                            value += pixel[2];
                            count++;
                        }

                        double ave_hue = hue / count;
                        double ave_saturation = saturation / count;
                        double ave_value = value / count;

                        // RCLCPP_INFO(this->get_logger(), "hue%f,saturation%f,value%f" , ave_hue, ave_saturation, ave_value);

                        if(court_color == "red"){
                            if(ave_value < 90 && ave_saturation < 190) msg_suction_check->data = ""; //吸引機構を認識している
                            else if((ave_hue >= 0 && ave_hue <= 10) || (ave_hue >= 100 && ave_hue <= 110) || (ave_hue >= 160 && ave_hue <= 180)) msg_suction_check->data = "R"; //赤ボール
                            else if(ave_hue >= 130) msg_suction_check->data = "P"; //紫ボール
                            else msg_suction_check->data = ""; //吸引機構を認識している
                        }
                        else if(court_color == "blue"){
                            if(ave_value < 90 && ave_saturation < 190) msg_suction_check->data = ""; //吸引機構を認識している
                            else if(ave_hue >= 110) msg_suction_check->data = "P"; //紫ボール
                            else if(ave_hue >= 90)msg_suction_check->data = "B"; //青ボール
                            else msg_suction_check->data = ""; //吸引機構を認識している
                        }
                    }

                    _pub_suction_check->publish(*msg_suction_check);

                    ////////////d435iを見る
                    // 点の座標を設定
                    // cv::Point front_point(front_suction_check_point[1], front_suction_check_point[0]);
                    // cv::Point back_point(back_suction_check_point[1], back_suction_check_point[0]);

                    // // 画像に緑色の点を描画（半径2の小さい円として）
                    // const cv::Scalar greenColor(0, 255, 0);  // BGRで緑色
                    // cv::circle(frame_rgb, front_point, 2, greenColor, -1);  // 塗りつぶしの円として描画
                    // cv::circle(frame_rgb, back_point, 2, greenColor, -1);  // 塗りつぶしの円として描画
                    // cv::Size target_size(1280, 720);
                    // cv::resize(frame_rgb, frame_prev, target_size);
                    // cv::imshow("d435i", frame_prev);
                    // cv::waitKey(1);
                    ///////////
                }
            }
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
                c1caera_c2ode_flag = true;
                silo_flag = true;
            }
        }

        void DetectionInterface::callback_now_sequence(const std_msgs::msg::String::SharedPtr msg){
            now_sequence = msg->data;
        }

        void DetectionInterface::callback_way_point(const std_msgs::msg::String::SharedPtr msg){
            way_point = msg->data;
            if(way_point == "c3_a" || way_point == "c3_b" || way_point == "c6_a" || way_point == "c6_b") c3_c4_flag = true;
            if(way_point == "c1") silo_flag = true;
        }    
}