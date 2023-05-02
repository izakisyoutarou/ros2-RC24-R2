#include "RANSAC_localization/detect_lines.hpp"

void DtectLines::setup(const string &robot_type, const double &voxel_size, const int &trial_num, const double &inlier_dist_threshold, const double &inlier_length_threshold){
  robot_type_ = robot_type;
  voxel_size_ = voxel_size;
  trial_num_ = trial_num;
  inlier_dist_threshold_ = inlier_dist_threshold;
  points_num_threshold = inlier_length_threshold / voxel_size_;
}

void DtectLines::init(){
  estimated_diff = Vector3d::Zero();
  sum.clear();
  for (int i=0; i<lines.size(); ++i){
    lines[i].clear();
    lines_[i].points.clear();
  }
  lines.clear();
  lines_.clear();
  if(robot_type_ == "ER"){
    lines.resize(8);
    lines_.resize(8);
  }
  else if(robot_type_ == "RR"){
    lines.resize(6);
    lines_.resize(6);
  }
}

void DtectLines::fuse_inliers(const vector<LaserPoint> &src_points){
  init();
  devide_points(src_points);
  get_inliers();
  if(robot_type_ == "RR" && lines_[1].points.size()>0) detect_circles_flag=true;
  calc_estimated_diff();
}


void DtectLines::calc_estimated_diff(){
  estimated_diff[2] = calc_diff_angle();
  for (size_t i=0; i<lines_.size(); i++){
    if(lines_[i].points.size()==0) continue;
    double average = calc_average(i);
    if(robot_type_ == "ER"){
      if(i<4) estimated_diff[1] = ER_map_point_y[i] - average;
      else estimated_diff[0] = ER_map_point_x[i-4] - average;
    }
    else if(robot_type_ == "RR"){
      if(i<4) estimated_diff[1] = RR_map_point[i] - average;
      else estimated_diff[0] = RR_map_point[i-4] - average;
    }
  }
  // 垂木だけを読むことでリング回収時の自己位置精度を高める
  if(robot_type_ == "ER"){
    check_tracking();
    if(is_tracking_rafter_right && is_tracking_rafter_left){
      if(lines_[0].points.size()<lines_[3].points.size()) is_tracking_rafter_right=false;
      else is_tracking_rafter_left=true;
    }
    if(is_tracking_rafter_right) calc_tracking_diff(0);
    if(is_tracking_rafter_left) calc_tracking_diff(3);
  }
}

void DtectLines::calc_tracking_diff(const int &num){
      if(!lines_[num].points.size()==0){
        estimated_diff[1] = ER_map_point_y[num] - calc_average(num);
        // estimated_diff[2] = lines_[num].angle;
      }
      else {
        estimated_diff[1]=0.0;
        // estimated_diff[2]=0.0;
      }
}

double DtectLines::calc_average(const int &num){
  double sum=0.0;
  for(size_t i=0; i<lines_[num].points.size(); i++){
    if(num<4) sum += lines_[num].points[i].y;
    else sum += lines_[num].points[i].x;
  }
  return sum/lines_[num].points.size();
}

double DtectLines::check_tracking(){
  if(!lines_[0].points.size()==0){
    is_tracking_rafter_right=true;
    detect_rafter_right_time = chrono::system_clock::now();
  }
  if(chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now()-detect_rafter_right_time).count()>1500){
    is_tracking_rafter_right=false;
  }
  if(!lines_[3].points.size()==0){
    is_tracking_rafter_left=true;
    detect_rafter_left_time = chrono::system_clock::now();
  }
  if(chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now()-detect_rafter_left_time).count()>1500){
    is_tracking_rafter_left=false;
  }
}

double DtectLines::calc_diff_angle(){
  double diff_angle=0.0;
  double best_diff_angle=100.0;
  int get_angle_count=0;
  for(size_t i=0; i<lines_.size(); i++){
    if(lines_[i].points.size()==0) continue;
    get_angle_count++;
    if(i<4) diff_angle = lines_[i].angle;
    else{
      if(lines_[i].angle<0) diff_angle = lines_[i].angle + M_PI/2;
      else diff_angle = lines_[i].angle - M_PI/2;
    }
    if(diff_angle < best_diff_angle) best_diff_angle = diff_angle;
  }
  if(get_angle_count==0) return 0.0;
  return best_diff_angle;
}

void DtectLines::devide_points(const vector<LaserPoint> &src_points){
  for(size_t i=0; i<src_points.size(); i++){
    if(robot_type_ == "ER"){
      set_points(lines[0], ER_map_point_x[0], ER_map_point_x[3], ER_map_point_y[0], ER_map_point_y[0], src_points[i]);
      set_points(lines[1], ER_map_point_x[1], ER_map_point_x[3]-bridge_width-distance_threshold, ER_map_point_y[1], ER_map_point_y[1], src_points[i]);
      set_points(lines[2], ER_map_point_x[1], ER_map_point_x[2], ER_map_point_y[2], ER_map_point_y[2], src_points[i]);
      set_points(lines[3], ER_map_point_x[0], ER_map_point_x[2], ER_map_point_y[3], ER_map_point_y[3], src_points[i]);
      set_points(lines[4], ER_map_point_x[0], ER_map_point_x[0], ER_map_point_y[0], ER_map_point_y[3], src_points[i]);
      set_points(lines[5], ER_map_point_x[1], ER_map_point_x[1], ER_map_point_y[1], ER_map_point_y[2], src_points[i]);
      set_points(lines[6], ER_map_point_x[3], ER_map_point_x[3], ER_map_point_y[0], ER_map_point_y[1], src_points[i]);
      set_points(lines[7], ER_map_point_x[2], ER_map_point_x[2], ER_map_point_y[2], ER_map_point_y[3], src_points[i]);
    }
    else if(robot_type_ == "RR"){
      set_points(lines[0], RR_map_point[0], RR_map_point[3], RR_map_point[0], RR_map_point[0], src_points[i]);
      set_points(lines[1], RR_map_point[1], RR_map_point[2], RR_map_point[1], RR_map_point[1], src_points[i]);
      set_points(lines[2], RR_map_point[1], RR_map_point[2], RR_map_point[2], RR_map_point[2], src_points[i]);
      set_points(lines[3], RR_map_point[0], RR_map_point[3], RR_map_point[3], RR_map_point[3], src_points[i]);
      set_points(lines[4], RR_map_point[0], RR_map_point[0], RR_map_point[0], RR_map_point[3], src_points[i]);
      set_points(lines[5], RR_map_point[1], RR_map_point[1], RR_map_point[1], RR_map_point[2], src_points[i]);
    }
  }
}

void DtectLines::get_inliers(){
  for(size_t i=0; i<lines.size(); i++){
    lines_[i]=calc_inliers(lines[i]);
    if(i<4) clear_points(lines_[i], 0, 30);
    else clear_points(lines_[i], 60, 90);
    input_points(lines_[i]);
  }
}

void DtectLines::set_points(vector<LaserPoint> &points, const double map_point_x_1, const double map_point_x_2, const double map_point_y_1, const double map_point_y_2, const LaserPoint &src_point){
  if(map_point_x_1 - distance_threshold < src_point.x &&
     map_point_x_2 + distance_threshold > src_point.x &&
     map_point_y_1 - distance_threshold < src_point.y &&
     map_point_y_2 + distance_threshold > src_point.y){
    LaserPoint point;
    point = src_point;
    points.push_back(point);
  }
  return;
}


void DtectLines::input_points(const EstimatedLine &line){
  LaserPoint sum_point;
  for(size_t i=0; i<line.points.size(); i++){
    sum_point.x = line.points[i].x;
    sum_point.y = line.points[i].y;
    sum.push_back(sum_point);
  }
}


EstimatedLine DtectLines::calc_inliers(vector<LaserPoint> &divided_points){
  EstimatedLine inlier;
  inlier.angle=0.0;
  if(divided_points.size()==0){
    return inlier;
  }
  int best_inlier_num=0;
  double best_diff_y=0;
  double best_diff_x=0;
  double best_diff_xy=0;
  double best_a = 0.0;
  double best_b = 0.0;
  double best_c = 0.0;
  // ランダムサンプリング
  for (int i = 0; i < trial_num_; i++) {
    // ランダムに2点を選択する
    const int p1_idx = static_cast<int>(rand() / (RAND_MAX + 1.0) * divided_points.size());
    const int p2_idx = static_cast<int>(rand() / (RAND_MAX + 1.0) * divided_points.size());
    // 直線の係数を計算する
    const double diff_y = divided_points[p2_idx].y - divided_points[p1_idx].y;
    const double diff_x = divided_points[p1_idx].x - divided_points[p2_idx].x;
    const double diff_xy = divided_points[p1_idx].y * divided_points[p2_idx].x - divided_points[p2_idx].y * divided_points[p1_idx].x;
    // インライア数を計算する
    int inlier_num = 0;
    for (const auto& point : divided_points) {
      const double dist = abs(diff_y * point.x + diff_x * point.y + diff_xy) / sqrt(diff_y * diff_y + diff_x * diff_x);
      if (dist < inlier_dist_threshold_) {
        inlier_num++;
      }
    }
    // 最尤推定値を更新する
    if (inlier_num > best_inlier_num) {
      best_diff_y = diff_y;
      best_diff_x = diff_x;
      best_diff_xy = diff_xy;
      best_inlier_num = inlier_num;
      best_a = best_diff_y/best_diff_x;
      best_b = -1;
      best_c = divided_points[p2_idx].y - best_a*divided_points[p2_idx].x;
    }
  }
  for(size_t i=0; i<best_inlier_num; i++){
    const double dist = abs(best_diff_y * divided_points[i].x + best_diff_x * divided_points[i].y + best_diff_xy) / sqrt(best_diff_y * best_diff_y + best_diff_x * best_diff_x);
    if (dist < inlier_dist_threshold_) {
      LaserPoint temp_point;
      temp_point.x = divided_points[i].x;
      temp_point.y = divided_points[i].y;
      inlier.points.push_back(temp_point);
    }
  }
  inlier.angle=atan(best_a);
  return inlier;
}

bool DtectLines::clear_points(EstimatedLine &estimated_line, int angle_threshold_min, int angle_threshold_max){
  if(estimated_line.points.size()<points_num_threshold || abs(radToDeg(estimated_line.angle)) > angle_threshold_max || abs(radToDeg(estimated_line.angle)) < angle_threshold_min){
    estimated_line.points.clear();
    return true;
  }
  return false;
}

double DtectLines::LPF(const double &raw){
  double k=0.1;
  double lpf = (1 - k) * last_lpf + k * raw;
  last_lpf = lpf;
  return lpf;
}

vector<LaserPoint> DtectLines::get_sum(){return sum;}
Vector3d DtectLines::get_estimated_diff(){return estimated_diff;}
