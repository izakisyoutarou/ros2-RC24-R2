#include "icp_base_slam/ransac_lines.hpp"

void RansacLines::fuse_inliers(vector<config::LaserPoint> &src_points, const Pose &estimated, double &odom_to_lidar_x, double &odom_to_lidar_y){
  init();
  devide_points(src_points);
  get_inliers();
  calc_estimated_diff(estimated, odom_to_lidar_x, odom_to_lidar_y);
}

void RansacLines::init(){
  estimated_diff = Pose();
  sum.clear();
  for (int i=0; i<lines.size(); ++i){
    lines[i].clear();
    lines_[i].points.clear();
  }
  lines.clear();
  lines_.clear();
  lines.resize(8);
  lines_.resize(8);
}


void RansacLines::calc_estimated_diff(const Pose &estimated, double &odom_to_lidar_x, double &odom_to_lidar_y){
  estimated_diff.yaw = calc_diff_angle();
  for (size_t i=0; i<lines_.size(); i++){
    if(lines_[i].points.size()==0) continue;
    double dist;
    double dist_sum=0.0;
    for(size_t j=0; j<lines_[i].points.size(); j++){
      if(i<4) dist_sum += lines_[i].points[j].dist * sin(lines_[i].points[j].angle + estimated_diff.yaw);
      else dist_sum += lines_[i].points[j].dist * cos(lines_[i].points[j].angle + estimated_diff.yaw);
    }
    dist = dist_sum / lines_[i].points.size();
    if(i<4) estimated_diff.y = -(estimated.y + odom_to_lidar_y - (dist + map_point_y[i]));
    else estimated_diff.x = -(estimated.x + odom_to_lidar_x - (-dist + map_point_x[i-4]));
  }
}

double RansacLines::calc_diff_angle(){
  double diff_angle=0.0;
  double angle=0.0;
  int get_angle_count=0;
  for(size_t i=0; i<lines_.size(); i++){
    if(lines_[i].points.size()==0) continue;
    get_angle_count++;
    if(i<4) angle = lines_[i].angle;
    else{
      if(lines_[i].angle<0) angle = lines_[i].angle + M_PI/2;
      else angle = lines_[i].angle - M_PI/2;
    }
    diff_angle += angle;
  }
  if(get_angle_count==0) return 0.0;
  return LPF(diff_angle/get_angle_count);
}

void RansacLines::devide_points(vector<config::LaserPoint> &src_points){
  for(size_t i=0; i<src_points.size(); i++){
    set_points(lines[0], map_point_x[0], map_point_x[2], map_point_y[0], map_point_y[0], src_points[i]);
    set_points(lines[1], map_point_x[1], map_point_x[2], map_point_y[1], map_point_y[1], src_points[i]);
    set_points(lines[2], map_point_x[1], map_point_x[2], map_point_y[2], map_point_y[2], src_points[i]);
    set_points(lines[3], map_point_x[0], map_point_x[2], map_point_y[3], map_point_y[3], src_points[i]);
    set_points(lines[4], map_point_x[0], map_point_x[0], map_point_y[0], map_point_y[3], src_points[i]);
    set_points(lines[5], map_point_x[1], map_point_x[1], map_point_y[1], map_point_y[2], src_points[i]);
    set_points(lines[6], map_point_x[2], map_point_x[2], map_point_y[0], map_point_y[1], src_points[i]);
    set_points(lines[7], map_point_x[2], map_point_x[2], map_point_y[2], map_point_y[3], src_points[i]);
  }
}

void RansacLines::get_inliers(){
  for(size_t i=0; i<lines.size(); i++){
    lines_[i]=calc_inliers(lines[i]);
    if(i<4) clear_points(lines_[i], 100, 0, 10);
    else clear_points(lines_[i], 100, 80, 90);
    input_points(lines_[i]);
  }
}

void RansacLines::set_points(vector<config::LaserPoint> &points, double map_point_x_1, double map_point_x_2, double map_point_y_1, double map_point_y_2, config::LaserPoint &src_point){
  if(map_point_x_1 - distance_threshold < src_point.x &&
     map_point_x_2 + distance_threshold > src_point.x &&
     map_point_y_1 - distance_threshold < src_point.y &&
     map_point_y_2 + distance_threshold > src_point.y){
    config::LaserPoint point;
    point = src_point;
    points.push_back(point);
  }
  return;
}


void RansacLines::input_points(EstimatedLine &line){
  config::LaserPoint sum_point;
  for(size_t i=0; i<line.points.size(); i++){
    sum_point.x = line.points[i].x;
    sum_point.y = line.points[i].y;
    sum.push_back(sum_point);
  }
}


EstimatedLine RansacLines::calc_inliers(vector<config::LaserPoint> &divided_points){
  inlier.points.clear();
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
    int p1_idx = static_cast<int>(rand() / (RAND_MAX + 1.0) * divided_points.size());
    int p2_idx = static_cast<int>(rand() / (RAND_MAX + 1.0) * divided_points.size());
    // 直線の係数を計算する
    double diff_y = divided_points[p2_idx].y - divided_points[p1_idx].y;
    double diff_x = divided_points[p1_idx].x - divided_points[p2_idx].x;
    double diff_xy = divided_points[p1_idx].y * divided_points[p2_idx].x - divided_points[p2_idx].y * divided_points[p1_idx].x;
    detect_length = sqrt(diff_x*diff_x + diff_y*diff_y);
    if(detect_length < 1.1) continue;
    // インライア数を計算する
    int inlier_num = 0;
    for (const auto& point : divided_points) {
      double dist = abs(diff_y * point.x + diff_x * point.y + diff_xy) / sqrt(diff_y * diff_y + diff_x * diff_x);
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
    double dist = abs(best_diff_y * divided_points[i].x + best_diff_x * divided_points[i].y + best_diff_xy) / sqrt(best_diff_y * best_diff_y + best_diff_x * best_diff_x);
    if (dist < inlier_dist_threshold_) {
      config::LaserPoint temp_point;
      temp_point.x = divided_points[i].x;
      temp_point.y = divided_points[i].y;
      temp_point.angle = divided_points[i].angle;
      temp_point.dist = divided_points[i].dist;
      inlier.points.push_back(temp_point);
    }
  }
  inlier.angle=atan(best_a);
  return inlier;
}

bool RansacLines::clear_points(EstimatedLine &estimated_line, int size_threshold, int angle_threshold_min, int angle_threshold_max){
  if(estimated_line.points.size()<size_threshold || abs(radToDeg(estimated_line.angle)) > angle_threshold_max || abs(radToDeg(estimated_line.angle)) < angle_threshold_min){
    estimated_line.points.clear();
    return true;
  }
  return false;
}

double RansacLines::LPF(double raw){
  double k=0.3;
  double lpf = (1 - k) * last_lpf + k * raw;
  last_lpf = lpf;
  return lpf;
}

vector<config::LaserPoint> RansacLines::get_sum(){return sum;}
Pose RansacLines::get_estimated_diff(){return estimated_diff;}
