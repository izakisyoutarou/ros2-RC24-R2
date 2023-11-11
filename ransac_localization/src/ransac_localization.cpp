#include "ransac_localization/ransac_localization.hpp"
#include <boost/format.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

namespace self_localization{
  ransaclocalization::ransaclocalization(const rclcpp::NodeOptions &options) : ransaclocalization("", options) {}

  ransaclocalization::ransaclocalization(const string& name_space, const rclcpp::NodeOptions &options)
  :  rclcpp::Node("rasnac_localization", name_space, options),
    initial_pose_(get_parameter("initial_pose").as_double_array()),
    second_initial_pose_(get_parameter("2nd_initial_pose").as_double_array()),
    tf_array(get_parameter("tf_laser2robot").as_double_array()),
    plot_mode_(get_parameter("plot_mode").as_bool()),
    laser_weight_(get_parameter("laser_weight").as_double()),
    odom_weight_liner_(get_parameter("odom_weight_liner").as_double()),
    odom_weight_angler_(get_parameter("odom_weight_angler").as_double()),
    voxel_size_(get_parameter("voxel_size").as_double()),
    trial_num_(get_parameter("trial_num").as_int()),
    inlier_dist_threshold_(get_parameter("inlier_dist_threshold").as_double()),
    inlier_length_threshold_(get_parameter("inlier_length_threshold").as_double()),
    odom_linear_can_id(get_parameter("canid.odom_linear").as_int()),
    odom_angular_can_id(get_parameter("canid.odom_angular").as_int()),
    init_angular_can_id(get_parameter("canid.init_angular").as_int()),
    court_color_(get_parameter("court_color").as_string()){

    RCLCPP_INFO(this->get_logger(), "START");
    restart_subscriber = this->create_subscription<controller_interface_msg::msg::BaseControl>(
      "pub_base_control",_qos,
      bind(&ransaclocalization::callback_restart, this, std::placeholders::_1));

    scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
    bind(&ransaclocalization::callback_scan, this, std::placeholders::_1));

    odom_linear_subscriber = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
      "can_rx_"+ (boost::format("%x") % odom_linear_can_id).str(),_qos,
      bind(&ransaclocalization::callback_odom_linear, this, std::placeholders::_1));

    odom_angular_subscriber = this->create_subscription<socketcan_interface_msg::msg::SocketcanIF>(
      "can_rx_"+ (boost::format("%x") % odom_angular_can_id).str(),_qos,
      bind(&ransaclocalization::callback_odom_angular, this, std::placeholders::_1));

    init_angle_publisher = this->create_publisher<socketcan_interface_msg::msg::SocketcanIF>(
      "can_tx",_qos);

    self_pose_publisher = this->create_publisher<geometry_msgs::msg::Vector3>(
      "self_pose", _qos);

    tf_laser2robot << tf_array[0], tf_array[1], tf_array[2], tf_array[3], tf_array[4], tf_array[5];
    court_color = court_color_;

    if(court_color == "blue"){
      init_pose << initial_pose_[0], initial_pose_[1], initial_pose_[2];
    }else if(court_color == "red"){
      init_pose << initial_pose_[0], -1.0*initial_pose_[1], initial_pose_[2];
    }

    init();

    vector<LineData> lines;
    load_map_config(lines,court_color);
    create_map(lines);

    detect_lines.setup(lines, voxel_size_, trial_num_, inlier_dist_threshold_, inlier_length_threshold_);
    pose_fuser.setup(lines, laser_weight_, odom_weight_liner_, odom_weight_angler_);
    voxel_grid_filter.setup(voxel_size_);

    if(plot_mode_){
      ransaced_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "self_localization/ransac",fast_qos);

      pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "self_localization/pose", fast_qos);

      odom_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "self_localization/odom", fast_qos);

      map_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "self_localization/map", fast_qos);
    }
  }

  void ransaclocalization::load_map_config(vector<LineData> &lines, const string color){
    if(color == "blue"){
      ifstream ifs(ament_index_cpp::get_package_share_directory("main_executor") + "/config/ransac_localization/lines_blue.cfg");
      string str;
      int line_count=0;
      while(getline(ifs, str)){
        string token;
        istringstream stream(str);
        int count = 0;
        LineData line_data;
        while(getline(stream, token, ' ')){   //istringstream型の入力文字列を半角空白で分割しtokenに格納
          if(line_count==0) break;
          if(count==0) line_data.p_1.x = stold(token); //文字として取得した値をdouble型に変換
          else if(count==1) line_data.p_1.y = stold(token);
          else if(count==2) line_data.p_2.x = stold(token);
          else if(count==3) line_data.p_2.y = stold(token);
          count++;
        }
        line_data.set_data();
        if(!(line_count==0)) lines.push_back(line_data);
        line_count++;
      }
    }else if(color == "red"){
      ifstream ifs(ament_index_cpp::get_package_share_directory("main_executor") + "/config/ransac_localization/lines_red.cfg");
      string str;
      int line_count=0;
      while(getline(ifs, str)){
        string token;
        istringstream stream(str);
        int count = 0;
        LineData line_data;
        while(getline(stream, token, ' ')){   //istringstream型の入力文字列を半角空白で分割しtokenに格納
          if(line_count==0) break;
          if(count==0) line_data.p_1.x = stold(token); //文字として取得した値をdouble型に変換
          else if(count==1) line_data.p_1.y = stold(token);
          else if(count==2) line_data.p_2.x = stold(token);
          else if(count==3) line_data.p_2.y = stold(token);
          count++;
        }
        line_data.set_data();
        if(!(line_count==0)) lines.push_back(line_data);
        line_count++;
      }
    }
    return;
  }

  void ransaclocalization::init(){
    odom = Vector3d::Zero();
    last_odom = Vector3d::Zero();
    diff_odom = Vector3d::Zero();
    est_diff_sum = init_pose;
    last_estimated = init_pose;
    pose_fuser.init();
    detect_lines.init();
  }

  void ransaclocalization::callback_restart(const controller_interface_msg::msg::BaseControl::SharedPtr msg){
    if(msg->is_restart){
      RCLCPP_INFO(this->get_logger(), "RESTART");
      // if(msg->initial_state=="O"){
      //   if(court_color == "blue"){
      //     init_pose << initial_pose_[0], initial_pose_[1], initial_pose_[2];
      //   }else if(court_color == "red"){
      //     init_pose << initial_pose_[0], -1*initial_pose_[1], initial_pose_[2];
      //   } 
      // }
      // else if(msg->initial_state=="P") init_pose << second_initial_pose_[0], second_initial_pose_[1], second_initial_pose_[2];

      init_flag=true;

      // 初期角度をpublish
      uint8_t _candata[8];
      auto msg_angle = std::make_shared<socketcan_interface_msg::msg::SocketcanIF>();
      msg_angle->canid = init_angular_can_id;
      msg_angle->candlc = 4;
      float_to_bytes(_candata, static_cast<float>(init_pose[2]));
      for(int i=0; i<msg_angle->candlc; i++) msg_angle->candata[i] = _candata[i];
      init_angle_publisher->publish(*msg_angle);
    }
  }

  void ransaclocalization::callback_odom_linear(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    double odom_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    double dt_odom = odom_received_time - last_odom_received_time;
    last_odom_received_time = odom_received_time;
    uint8_t _candata[8];
    for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
    const double x = (double)bytes_to_float(_candata); //下層からのオドメトリx軸
    const double y = (double)bytes_to_float(_candata+4); //下層からのオドメトリy軸

    //cout<<x<<"  "<<y<<endl;

    diff_odom[0] = x - last_odom[0];//x軸の移動距離の差
    diff_odom[1] = y - last_odom[1];//y軸の移動距離の差

    if(abs(diff_odom[0]) / dt_odom > 7) diff_odom[0] = 0.0;//オドメトリが飛んだときの対策
    if(abs(diff_odom[1]) / dt_odom > 7) diff_odom[1] = 0.0;

    odom[0] += diff_odom[0];
    odom[1] += diff_odom[1];
    last_odom[0] = x;
    last_odom[1] = y;
  }

  void ransaclocalization::callback_odom_angular(const socketcan_interface_msg::msg::SocketcanIF::SharedPtr msg){
    //cout<<initial_pose_[0]<<" "<<initial_pose_[1]<<" "<<initial_pose_[2]<<endl;
    double jy_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    if(init_jy_time_flag) last_jy_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    init_jy_time_flag=false;
    double dt_jy = jy_received_time - last_jy_received_time;
    last_jy_received_time = jy_received_time;
    uint8_t _candata[8];
    for(int i=0; i<msg->candlc; i++) _candata[i] = msg->candata[i];
    const double yaw = (double)bytes_to_float(_candata);//下層からの角度
    //cout<<yaw<<endl;
    diff_odom[2] = yaw - last_odom[2];
    if(abs(diff_odom[2]) / dt_jy > 6*M_PI) diff_odom[2] = 0.0; //ジャイロが飛んだときの対策
    odom[2] += diff_odom[2];
    last_odom[2] = yaw;

    if(init_flag){
      init_flag=false;
      init();
    }

    vector_msg.x = odom[0] + est_diff_sum[0];
    vector_msg.y = odom[1] + est_diff_sum[1];
    vector_msg.z = odom[2] + est_diff_sum[2];
    cout<<"x "<<vector_msg.x<<"   y "<<vector_msg.y<<"   z "<<vector_msg.z<<endl;
    //cout<<"x "<<odom[0]+init_pose[0]<<"  y "<<odom[1]+init_pose[1]<<"  z "<<odom[2]+init_pose[2]<<endl;
    self_pose_publisher->publish(vector_msg);
  }

  void ransaclocalization::callback_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    time_start = chrono::system_clock::now();
    double current_scan_received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    dt_scan = current_scan_received_time - last_scan_received_time;
    last_scan_received_time = current_scan_received_time;

    Vector3d current_scan_odom = odom + est_diff_sum;
    Vector3d scan_odom_motion = current_scan_odom - last_estimated; //前回scanからのオドメトリ移動量

    if (scan_odom_motion[2] > M_PI) last_estimated[2] += 2*M_PI; //Yaw角度を-πからπの範囲で表現
    else if (scan_odom_motion[2] < -M_PI) last_estimated[2] -= 2*M_PI;
    scan_odom_motion[2] = current_scan_odom[2] - last_estimated[2];

    Vector3d laser = current_scan_odom + transform_sensor_position(tf_laser2robot); //lidarの取り付け位置を回転行列した値をオドメトリの座標に足している

    vector<LaserPoint> src_points = converter.scan_to_vector(msg, laser); //点群格納
    vector<LaserPoint> filtered_points = voxel_grid_filter.apply_voxel_grid_filter(src_points); //点群を等間隔に補正(yamlのvoxel_sizeで変更可能)
    //cout<<src_points.size()<<endl;

    detect_lines.fuse_inliers(filtered_points, laser);
    vector<LaserPoint> line_points = detect_lines.get_sum();
    trans = detect_lines.get_estimated_diff();

    linear_vel = distance(0.0,scan_odom_motion[0],0.0,scan_odom_motion[1]) / dt_scan;
    angular_vel = abs(scan_odom_motion[2]/dt_scan);

    Vector3d estimated = pose_fuser.fuse_pose(trans, current_scan_odom, linear_vel, angular_vel, line_points);

    Vector3d est_diff = estimated - current_scan_odom;  //直線からの推定値がデフォルト
    est_diff_sum += est_diff;
    last_estimated = estimated;
    if(plot_mode_) publishers(line_points);
    time_end = chrono::system_clock::now();

    //cout<<est_diff_sum[0]<<endl;
    //RCLCPP_INFO(this->get_logger(), "estimated x>%f y>%f a>%f°", estimated[0], estimated[1], radToDeg(estimated[2]));
    //RCLCPP_INFO(this->get_logger(), "trans x>%f y>%f a>%f°", trans[0], trans[1], radToDeg(trans[2]));
    //RCLCPP_INFO(this->get_logger(), "scan time->%d", chrono::duration_cast<chrono::milliseconds>(time_end-time_start).count());
  }

  void ransaclocalization::publishers(vector<LaserPoint> &points){
    sensor_msgs::msg::PointCloud2 cloud = converter.vector_to_PC2(points);

    corrent_pose_stamped.header.stamp = this->now();
    corrent_pose_stamped.header.frame_id = "map";
    corrent_pose_stamped.pose.position.x = odom[0] + est_diff_sum[0];
    corrent_pose_stamped.pose.position.y = odom[1] + est_diff_sum[1];
    corrent_pose_stamped.pose.orientation.z = sin((odom[2]+est_diff_sum[2]) / 2.0);
    corrent_pose_stamped.pose.orientation.w = cos((odom[2]+est_diff_sum[2]) / 2.0);

    odom_stamped.header.stamp = this->now();
    odom_stamped.header.frame_id = "map";
    odom_stamped.pose.position.x = odom[0] + init_pose[0];
    odom_stamped.pose.position.y = odom[1] + init_pose[1];
    odom_stamped.pose.orientation.z = sin((odom[2] +init_pose[2])/ 2.0);
    odom_stamped.pose.orientation.w = cos((odom[2] +init_pose[2]) / 2.0);

    map_publisher->publish(map_cloud);
    ransaced_publisher->publish(cloud);
    pose_publisher->publish(corrent_pose_stamped);
    odom_publisher->publish(odom_stamped);
  }

  void ransaclocalization::create_map(vector<LineData> &lines){
    LaserPoint map_point;
    for(size_t i=0; i<lines.size(); i++){
      if(lines[i].axis=='x'){
        for(int j=0; j<=static_cast<int>((lines[i].p_2.x - lines[i].p_1.x)*1000); j++){
          map_point.x = static_cast<double>(j)/1000 + lines[i].p_1.x;
          map_point.y = lines[i].p_1.y;
          map_points.push_back(map_point);
        }
      }
      else if(lines[i].axis=='y'){
        for(int j=0; j<=int((lines[i].p_2.y - lines[i].p_1.y)*1000); j++){
          map_point.y = static_cast<double>(j)/1000 + lines[i].p_1.y;
          map_point.x = lines[i].p_1.x;
          map_points.push_back(map_point);
        }
      }
    }
    map_cloud = converter.vector_to_PC2(map_points);
  }
}