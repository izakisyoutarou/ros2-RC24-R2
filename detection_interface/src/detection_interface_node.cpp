#include "detection_interface/detection_interface_node.hpp"

namespace detection_interface
{
    DetectionInterface::DetectionInterface(const rclcpp::NodeOptions &options) : DetectionInterface("", options) {}
    DetectionInterface::DetectionInterface(const std::string &name_space, const rclcpp::NodeOptions &options)
        : rclcpp::Node("detection_interface_node", name_space, options),

        suction_check_point(get_parameter("suction_check_point").as_integer_array()),
        depth_suction_check_value(get_parameter("depth_suction_check_value").as_int()),

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

        void DetectionInterface::callback_c1(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg){
            {
                std::lock_guard<std::mutex> lock(c1camera_mutex); // ミューテックスで保護
                if(now_sequence == "silo" && way_point == "c1" /*&& is_c1camera_callback*/){
                    // n++;
                    is_c1camera_callback = false;

                    // time_start = chrono::system_clock::now();
                    auto msg_collection_point = std::make_shared<std_msgs::msg::String>();

                    int silo_count = 0;

                    int max_size = 0;
                    int max_index = 0;

                    bool c1camera_count_flag = false;
                    bool c1camera_ball_flag = false;
                    
                    //c1カメラから受け取った情報を入れる配列
                    std::vector<std::string> class_id_c1camera;//redballとかblueballとか
                    std::vector<int> center_x_c1camera;//バウンディングボックスの真ん中(x)
                    std::vector<int> center_y_c1camera;//バウンディングボックスの真ん中(y)
                    std::vector<int> bbounbox_size_c1camera;//バウンディングの面積
                    std::vector<int> ymax_c1camera;//ボールのバウンディングの右下(y)

                    // std::array<std::vector<std::string>, 5> class_id_c1camera_list2;
                    // std::array<std::vector<int>, 5> center_x_c1camera_list2;
                    // std::array<std::vector<int>, 5> center_y_c1camera_list2;
                    // std::array<std::vector<int>, 5> bbounbox_size_c1camera_list2;
                    // std::array<std::vector<int>, 5> ymax_c1camera_list2;

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
                            // class_id_c1camera_list[c1camera_count].push_back(box.class_id);

                            // int size = static_cast<int>((box.xmax - box.xmin) * (box.ymax - box.ymin));
                            // bbounbox_size_c1camera_list[c1camera_count].push_back(size);

                            // int center_x_value = static_cast<int>((box.xmax + box.xmin) / 2);
                            // int center_y_value = static_cast<int>((box.ymax + box.ymin) / 2);
                            // center_x_c1camera_list[c1camera_count].push_back(center_x_value);
                            // center_y_c1camera_list[c1camera_count].push_back(center_y_value);

                            // ymax_c1camera_list[c1camera_count].push_back(box.ymax);

                            // c1camera_count_flag = true;


                            
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

                    // for (int z = 0; z < center_x_c1camera.size(); ++z) {
                    //     RCLCPP_INFO(this->get_logger(), "まえ%dceter_x%d", z, center_x_c1camera_list[c1camera_count][z]);
                    //     std::cout << class_id_c1camera_list[c1camera_count][z] << std::endl;
                    // }
                    
                    // for (int z = 0; z < center_x.size(); ++z) {
                    //     RCLCPP_INFO(this->get_logger(), "まえ%d@%d", z, center_x[z]);
                    // }

                    // for (int z = 0; z < count; ++z) {
                    //     RCLCPP_INFO(this->get_logger(), "Index: %d, xmin: %d, xmax: %d, ymin: %d, ymax: %d",
                    //                 z, min_max_xy[z][0], min_max_xy[z][1], min_max_xy[z][2], min_max_xy[z][3]);
                    // }

                    // if(c1camera_count_flag) c1camera_count++;

                    // RCLCPP_INFO(this->get_logger(), "まえ%d@%d", c1camera_count, n);

                    //ボールを5回検出した中で一番検出数が大きいものを選ぶ
                    // if(c1camera_count >= 5){
                    //     // for (int i = 0; i < 5; ++i) {
                    //     //     for (const auto& item : class_id_c1camera_list[i]) {
                    //     //         class_id_c1camera_list2[i].push_back(item);
                    //     //     }
                    //     //     for (const auto& item : center_x_c1camera_list[i]) {
                    //     //         center_x_c1camera_list2[i].push_back(item);
                    //     //     }
                    //     //     for (const auto& item : center_y_c1camera_list[i]) {
                    //     //         center_y_c1camera_list2[i].push_back(item);
                    //     //     }
                    //     //     for (const auto& item : bbounbox_size_c1camera_list[i]) {
                    //     //         bbounbox_size_c1camera_list2[i].push_back(item);
                    //     //     }
                    //     //     for (const auto& item : ymax_c1camera_list[i]) {
                    //     //         ymax_c1camera_list2[i].push_back(item);
                    //     //     }
                    //     // }

                    //     c1camera_count = 0;//5回推論したかどうかcountの初期化

                    //     //関数に渡すvector変数の初期化
                    //     for (int i = 0; i < ymax_c1camera.size(); ++i) {
                    //         class_id_c1camera.clear();
                    //         center_x_c1camera.clear();
                    //         center_y_c1camera.clear();
                    //         bbounbox_size_c1camera.clear();
                    //         ymax_c1camera.clear();
                    //     }

                    //     //5回の推論結果から一番認識数が多いindexを探す
                    //     for (int i = 0; i < ymax_c1camera_list.size(); ++i) {                        
                    //         if (ymax_c1camera_list[i].size() >= max_size) {
                    //             max_size = ymax_c1camera_list[i].size();
                    //             max_index = i;
                    //         }
                    //     }

                    //     //関数に渡すvector変数に一番認識数が多いindexを代入
                    //     class_id_c1camera.insert(class_id_c1camera.end(), class_id_c1camera_list[max_index].begin(), class_id_c1camera_list[max_index].end());
                    //     center_x_c1camera.insert(center_x_c1camera.end(), center_x_c1camera_list[max_index].begin(), center_x_c1camera_list[max_index].end());
                    //     center_y_c1camera.insert(center_y_c1camera.end(), center_y_c1camera_list[max_index].begin(), center_y_c1camera_list[max_index].end());
                    //     bbounbox_size_c1camera.insert(bbounbox_size_c1camera.end(), bbounbox_size_c1camera_list[max_index].begin(), bbounbox_size_c1camera_list[max_index].end());
                    //     ymax_c1camera.insert(ymax_c1camera.end(), ymax_c1camera_list[max_index].begin(), ymax_c1camera_list[max_index].end());

                    //     //5回の推論結果用のvector変数の初期化
                    //     for (int i = 0; i < ymax_c1camera_list.size(); ++i) {
                    //         center_x_c1camera_list[i].clear();
                    //         center_y_c1camera_list[i].clear();
                    //         bbounbox_size_c1camera_list[i].clear();
                    //         ymax_c1camera_list[i].clear();
                    //     }

                    //     c1camera_ball_flag = true;
                    // }

                    // for (int z = 0; z < center_x_c1camera.size(); ++z) {
                    //     RCLCPP_INFO(this->get_logger(), "まえ%dceter_x%d", z, center_x_c1camera[z]);
                    //     std::cout << class_id_c1camera[z] << std::endl;
                    // }

                    if(silo_count == 5 /*&& c1camera_ball_flag*/){//サイロが5個検出できたとき
                        if(now_sequence == "silo"){
                            if(way_point == "c1")c1camera_c1(ymax_c1camera, min_max_xy, center_x_c1camera, center_y_c1camera, bbounbox_size_c1camera, class_id_c1camera);
                        }
                    }

                    is_c1camera_callback = true;

                    // time_end = chrono::system_clock::now();
                    // RCLCPP_INFO(this->get_logger(), "scan time->%d[ms]", chrono::duration_cast<chrono::microseconds>(time_end-time_start).count());
                }
            }
        }

        void DetectionInterface::c1camera_c1(   const std::vector<int> ymax, std::array<std::array<int, 4>, 5>& min_max_xy,
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

        void DetectionInterface::callback_realsense(const bboxes_ex_msgs::msg::BoundingBoxes::SharedPtr msg){
            if(now_sequence == "storage" || now_sequence == "collect"){
                auto msg_collection_point = std::make_shared<std_msgs::msg::String>();
                auto msg_front_ball = std::make_shared<std_msgs::msg::Bool>();

                bool realsense_count_flag = false;

                int max_size = 0;
                int max_index = 0;

                //realsenseから受け取った情報を入れる配列
                std::vector<int> center_x_realsense;//バウンディングボックスの真ん中(x)
                std::vector<int> center_y_realsense;//バウンディングボックスの真ん中(y)
                std::vector<int> center_depth_realsense;//バウンディングボックスの真ん中のdepth(y)
                std::vector<int> ymax_realsense;//ボールのバウンディングの右下(y)

                silo_flag = true;

                for (const auto& box : msg->bounding_boxes) {
                    if(box.class_id == "redball" || box.class_id == "blueball"){
                        int center_x_value = static_cast<int>((box.xmax + box.xmin) / 2);
                        int center_y_value = static_cast<int>((box.ymax + box.ymin) / 2);
                        // center_x_realsense_list[realsense_count].push_back(center_x_value);
                        // center_y_realsense_list[realsense_count].push_back(center_y_value);
                        // center_depth_realsense_list[realsense_count].push_back(box.center_dist);
                        // ymax_realsense_list[realsense_count].push_back(box.ymax);

                        center_x_realsense.push_back(center_x_value);
                        center_y_realsense.push_back(center_y_value);
                        center_depth_realsense.push_back(box.center_dist);
                        ymax_realsense.push_back(box.ymax);

                        // realsense_count_flag = true;
                    }
                }

                // for (int z = 0; z < center_depth.size(); ++z) {
                //     RCLCPP_INFO(this->get_logger(), "まえ%d@%d", z, center_depth[z]);
                // }

                // if(realsense_count_flag) realsense_count++;

                // //ボールを5回検出した中で一番検出数が大きいものを選ぶ
                // if(realsense_count >= 5){
                //     realsense_count = 0;//5回推論したかどうかcountの初期化

                //    // 関数に渡すvector変数の初期化
                //     center_x_realsense.clear();
                //     center_y_realsense.clear();
                //     center_depth_realsense.clear();
                //     ymax_realsense.clear();

                //     //5回の推論結果から一番認識数が多いindexを探す
                //     for (int i = 0; i < center_x_realsense_list.size(); ++i) {                        
                //         if (center_x_realsense_list[i].size() > max_size) {
                //             max_size = center_x_realsense_list[i].size();
                //             max_index = i;
                //         }
                //     }

                //     //関数に渡すvector変数に一番認識数が多いindexを代入
                //     center_x_realsense.insert(center_x_realsense.end(), center_x_realsense_list[max_index].begin(), center_x_realsense_list[max_index].end());
                //     center_y_realsense.insert(center_y_realsense.end(), center_y_realsense_list[max_index].begin(), center_y_realsense_list[max_index].end());
                //     center_depth_realsense.insert(center_depth_realsense.end(), center_depth_realsense_list[max_index].begin(), center_depth_realsense_list[max_index].end());
                //     ymax_realsense.insert(ymax_realsense.end(), ymax_realsense_list[max_index].begin(), ymax_realsense_list[max_index].end());


                //     //5回の推論結果用のvector変数の初期化
                //     for (int i = 0; i < center_x_realsense_list.size(); ++i) {
                //         center_x_realsense_list[i].clear();
                //         center_y_realsense_list[i].clear();
                //         center_depth_realsense_list[i].clear();
                //         ymax_realsense_list[i].clear();
                //     }
                // }

                // for (int z = 0; z < center_depth_realsense.size(); ++z) {
                //     RCLCPP_INFO(this->get_logger(), "まえ%d@%d", z, center_depth_realsense[z]);
                // }

                if(center_x_realsense.size() != 0){//何も認識していないときに、下にいったらエラーを出す。そのためのif文
                    // //realseneのc3、c4から見たとき、どこのSTに行くか。ボールが手前か奥かaZZ
                    if(way_point == "c3" || way_point == "c6") realsense_c3_c4(center_x_realsense, center_y_realsense, center_depth_realsense, ymax_realsense);
                }
            }
        }

        void DetectionInterface::realsense_c3_c4(const std::vector<int> center_x, std::vector<int> center_y, const std::vector<int> center_depth, const std::vector<int> ymax){
            auto msg_ball_coordinate = std::make_shared<geometry_msgs::msg::Vector3>();

            // 最大値のイテレータを取得
            int max_it = *std::max_element(ymax.begin(), ymax.end());
            int threshold = max_it - 160;

            std::vector<std::vector<int>> xy_realsense_list;

            for(size_t i = 0; i < center_x.size(); i++){
                if(center_y[i] > threshold){
                    xy_realsense_list.push_back({center_x[i], center_y[i], center_depth[i]});
                    RCLCPP_INFO(this->get_logger(), "depth%d" ,center_depth[i]);
                }
            }

            int target_value = 640;
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

        void DetectionInterface::d435iImageCallback(const realsense2_camera_msgs::msg::RGBD::ConstSharedPtr &ptr){
            auto img_rgb = cv_bridge::toCvCopy(ptr->rgb, "bgr8");
            auto img_depth = cv_bridge::toCvCopy(ptr->depth, sensor_msgs::image_encodings::TYPE_16UC1);

            cv::Mat frame_rgb = img_rgb->image;
            cv::Mat frame_depth = img_depth->image;
            cv::Mat frame_prev;

            //ST系に入ったときにボールの吸着判定
            auto msg_suction_check = std::make_shared<std_msgs::msg::String>();
            cv::Vec3b pixel_value;
            cv::Vec3b average_color(0, 0, 0);
            std::vector<cv::Vec3b> colors;
            // std::array<std::array<cv::Vec3b, 3>, 9> colors2;

            uint16_t depth_value;
            uint16_t depth_average;
            int n = 0;

            std::array<cv::Vec3b, 9> colors2= {
                frame_rgb.at<cv::Vec3b>(suction_check_point[0], suction_check_point[1]), 
                frame_rgb.at<cv::Vec3b>(suction_check_point[0]-1, suction_check_point[1]), 
                frame_rgb.at<cv::Vec3b>(suction_check_point[0]-1, suction_check_point[1]+1), 
                frame_rgb.at<cv::Vec3b>(suction_check_point[0], suction_check_point[1]+1),
                frame_rgb.at<cv::Vec3b>(suction_check_point[0]+1, suction_check_point[1]+1), 
                frame_rgb.at<cv::Vec3b>(suction_check_point[0]+1, suction_check_point[1]),
                frame_rgb.at<cv::Vec3b>(suction_check_point[0]+1, suction_check_point[1]-1), 
                frame_rgb.at<cv::Vec3b>(suction_check_point[0], suction_check_point[1]-1),
                frame_rgb.at<cv::Vec3b>(suction_check_point[0]-1, suction_check_point[1]-1)
            };

            // RCLCPP_INFO(this->get_logger(), "まえ----[0]%d[1]%d[2]%d" , colors2[0][0], colors2[0][1], colors2[0][2]);

            // for (int i=0; i < colors2.size(); i++) {
            //     average_color[0] += colors2[i][0];
            //     average_color[1] += colors2[i][1];
            //     average_color[2] += colors2[i][2];
            // }

            // average_color[0] /= static_cast<int>(colors2.size());
            // average_color[1] /= static_cast<int>(colors2.size());
            // average_color[2] /= static_cast<int>(colors2.size());

            // std::cout << "Average Color - B: " << static_cast<int>(average_color[0]) << " G: " << static_cast<int>(average_color[1]) << " R: " << static_cast<int>(average_color[2]) << std::endl;

            std::array<int, 9> blue_values, green_values, red_values;
            for (size_t i = 0; i < colors2.size(); ++i) {
                blue_values[i] = colors2[i][0];
                green_values[i] = colors2[i][1];
                red_values[i] = colors2[i][2];
            }

            // 各チャンネルごとにソート
            std::sort(blue_values.begin(), blue_values.end());
            std::sort(green_values.begin(), green_values.end());
            std::sort(red_values.begin(), red_values.end());

            // 中央値を計算
            cv::Vec3b median_color;
            median_color[0] = blue_values[blue_values.size() / 2];
            median_color[1] = green_values[green_values.size() / 2];
            median_color[2] = red_values[red_values.size() / 2];

            // 中央値を表示
            std::cout << "Median Color - B: " << static_cast<int>(median_color[0]) 
                    << " G: " << static_cast<int>(median_color[1]) 
                    << " R: " << static_cast<int>(median_color[2]) << std::endl;

            // for (const auto& row : colors2) {
            //     for (const auto& color : row) {
            //         average_color[0] += color[0]; // Blueチャネルを加算
            //         average_color[1] += color[1]; // Greenチャネルを加算
            //         average_color[2] += color[2]; // Redチャネルを加算
            //     }
            // }

            // average_color[0] /= colors.size();
            // average_color[1] /= colors.size();
            // average_color[2] /= colors.size();

            // RCLCPP_INFO(this->get_logger(), "まえ----[0]%d[1]%d[2]%d" , average_color[0], average_color[1], average_color[2]);

            // //depthの平均値を計算
            // std::vector<uint16_t> center_dist = {   
            //     frame_depth.at<uint16_t>(suction_check_point[0], suction_check_point[1]), 
            //     frame_depth.at<uint16_t>(suction_check_point[0]-1, suction_check_point[1]), 
            //     frame_depth.at<uint16_t>(suction_check_point[0]-1, suction_check_point[1]+1), 
            //     frame_depth.at<uint16_t>(suction_check_point[0], suction_check_point[1]+1),
            //     frame_depth.at<uint16_t>(suction_check_point[0]+1, suction_check_point[1]+1), 
            //     frame_depth.at<uint16_t>(suction_check_point[0]+1, suction_check_point[1]),
            //     frame_depth.at<uint16_t>(suction_check_point[0]+1, suction_check_point[1]-1), 
            //     frame_depth.at<uint16_t>(suction_check_point[0], suction_check_point[1]-1),
            //     frame_depth.at<uint16_t>(suction_check_point[0]-1, suction_check_point[1]-1)
            // };

            // center_dist.erase(std::remove(center_dist.begin(), center_dist.end(), 0), center_dist.end());

            // // ゼロを削除した後、配列が空でない場合は平均を計算
            // if (!center_dist.empty()) {
            //     uint16_t sum = std::accumulate(center_dist.begin(), center_dist.end(), static_cast<uint16_t>(0));
            //     depth_average = sum / center_dist.size();
            //     // RCLCPP_INFO(this->get_logger(), "まえ%d" , depth_average);
            // }

            // if(depth_average < depth_suction_check_value) {
            //     uchar blue = average_color[0];
            //     uchar green = average_color[1];
            //     uchar red = average_color[2];

            //     if(red > green + 50 && red > blue + 50) msg_suction_check->data = "R"; //赤ボール
            //     else if(blue > green + 30 && blue > red + 30) msg_suction_check->data = "B"; //青ボール
            //     else if(red > green + 20 && blue > green + 20) msg_suction_check->data = "P"; //紫ボール
            // }
            // else {
            //     msg_suction_check->data = ""; //何も吸着できていない
            // }

            // _pub_suction_check->publish(*msg_suction_check);




            // pixel_value = frame_rgb.at<cv::Vec3b>(suction_check_point[0], suction_check_point[1]-1);

            // RCLCPP_INFO(this->get_logger(), "あと----[0]%d[1]%d[2]%d" , pixel_value[0], pixel_value[1], pixel_value[2]);
            // depth_value = frame_depth.at<uint16_t>(suction_check_point[0], suction_check_point[1]);

            // if(depth_value < depth_suction_check_value) {
            //     uchar blue = pixel_value[0];
            //     uchar green = pixel_value[1];
            //     uchar red = pixel_value[2];

            //     if(red > green + 50 && red > blue + 50) msg_suction_check->data = "R"; //赤ボール
            //     else if(blue > green + 30 && blue > red + 30) msg_suction_check->data = "B"; //青ボール
            //     else if(red > green + 20 && blue > green + 20) msg_suction_check->data = "P"; //紫ボール
            // }
            // else {
            //     msg_suction_check->data = ""; //何も吸着できていない
            // }

            // _pub_suction_check->publish(*msg_suction_check);

            ////////////d435iを見る(これは常に見てる)
            // 点の座標を設定
            cv::Point front_point(suction_check_point[1], suction_check_point[0]);

            // 画像に緑色の点を描画（半径2の小さい円として）
            const cv::Scalar greenColor(0, 255, 0);  // BGRで緑色
            cv::circle(frame_rgb, front_point, 2, greenColor, -1);  // 塗りつぶしの円として描画

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
            if(way_point == "c3")c3_c4_flag = true;
            if(way_point == "c6")c3_c4_flag = true;
            if(way_point == "c1")silo_flag = true;
        }    
}