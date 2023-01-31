#include "icp_base_slam/ransac_lines.hpp"

vector<config::LaserPoint> sum_points;
vector<config::LaserPoint> rafter_right_points;
vector<config::LaserPoint> rafter_right_points_;
vector<config::LaserPoint> rafter_left_points;
vector<config::LaserPoint> rafter_left_points_;
vector<config::LaserPoint> rafter_back_points;
vector<config::LaserPoint> rafter_back_points_;
vector<config::LaserPoint> fence_centor_points_right;
vector<config::LaserPoint> fence_centor_points_right_;
vector<config::LaserPoint> fence_centor_points_left;
vector<config::LaserPoint> fence_centor_points_left_;
vector<config::LaserPoint> fence_front_points;
vector<config::LaserPoint> fence_front_points_;
vector<config::LaserPoint> fence_right_points;
vector<config::LaserPoint> fence_right_points_;
vector<config::LaserPoint> fence_left_points;
vector<config::LaserPoint> fence_left_points_;
vector<config::LaserPoint> inlier_points;
int trial_num_;
double inlier_dist_threshold_;
double best_a = 0.0;
double best_b = 0.0;
double best_c = 0.0;
double detect_length = 0.0;
double est_x=0.0;
double est_y=0.0;
double distance_threshold=0.8;
chrono::system_clock::time_point time_start, time_end;

void RansacLines::fuse_inliers(PclCloud &src_cloud, int trial_num, double inlier_dist_threshold, const Pose &estimated){
  trial_num_=trial_num;
  inlier_dist_threshold_=inlier_dist_threshold;
  est_x = estimated.x;
  est_y = estimated.y;
  sum_points.clear();
  rafter_right_points.clear();
  rafter_right_points_.clear();
  rafter_left_points.clear();
  rafter_left_points_.clear();
  rafter_back_points.clear();
  rafter_back_points_.clear();
  fence_centor_points_right.clear();
  fence_centor_points_right_.clear();
  fence_centor_points_left.clear();
  fence_centor_points_left_.clear();
  fence_front_points.clear();
  fence_front_points_.clear();
  fence_right_points.clear();
  fence_right_points_.clear();
  fence_left_points.clear();
  fence_left_points_.clear();

  for(size_t i=0; i<src_cloud.points.size(); i++){
    set_points(rafter_right_points, map_point_x[0], map_point_x[2], map_point_y[0], map_point_y[0], src_cloud.points[i], i);
    set_points(rafter_left_points, map_point_x[0], map_point_x[2], map_point_y[3], map_point_y[3], src_cloud.points[i], i);
    set_points(rafter_back_points, map_point_x[0], map_point_x[0], map_point_y[0], map_point_y[3], src_cloud.points[i], i);
    set_points(fence_centor_points_right, map_point_x[2], map_point_x[2], map_point_y[0], map_point_y[1], src_cloud.points[i], i);
    set_points(fence_centor_points_left, map_point_x[2], map_point_x[2], map_point_y[2], map_point_y[3], src_cloud.points[i], i);
    set_points(fence_front_points, map_point_x[1], map_point_x[1], map_point_y[1], map_point_y[2], src_cloud.points[i], i);
    set_points(fence_right_points, map_point_x[1], map_point_x[2], map_point_y[1], map_point_y[1], src_cloud.points[i], i);
    set_points(fence_left_points, map_point_x[1], map_point_x[2], map_point_y[2], map_point_y[2], src_cloud.points[i], i);
  }
  rafter_right_points_=get_inlier(rafter_right_points);
  clear_points(rafter_right_points_, 100, 0, 20);
  // if(rafter_right_points_.size() > 100) cout << (tan(best_a)) << "°" << endl;
  rafter_left_points_=get_inlier(rafter_left_points);
  clear_points(rafter_right_points_, 100, 0, 20);
  rafter_back_points_=get_inlier(rafter_back_points);
  clear_points(rafter_back_points_, 150, 70, 90);
  fence_centor_points_right_=get_inlier(fence_centor_points_right);
  clear_points(fence_centor_points_right_, 100, 70, 90);
  fence_centor_points_left_=get_inlier(fence_centor_points_left);
  clear_points(fence_centor_points_left_, 100, 70, 90);
  fence_front_points_=get_inlier(fence_front_points);
  clear_points(fence_front_points_, 100, 70, 90);
  fence_right_points_=get_inlier(fence_right_points);
  clear_points(fence_right_points_, 100, 0, 20);
  fence_left_points_=get_inlier(fence_left_points);
  clear_points(fence_left_points_, 100, 0, 20);
  input_points(rafter_right_points_);
  input_points(rafter_left_points_);
  input_points(rafter_back_points_);
  input_points(fence_centor_points_right_);
  input_points(fence_centor_points_left_);
  input_points(fence_front_points_);
  input_points(fence_right_points_);
  input_points(fence_left_points_);
}

void RansacLines::set_points(vector<config::LaserPoint> &points, double map_point_x_1, double map_point_x_2, double map_point_y_1, double map_point_y_2, PointType src_point, int i){
  if(map_point_x_1 - distance_threshold < src_point.x &&
     map_point_x_2 + distance_threshold > src_point.x &&
     map_point_y_1 - distance_threshold < src_point.y &&
     map_point_y_2 + distance_threshold > src_point.y){
    config::LaserPoint point;
    point.x = src_point.x;
    point.y = src_point.y;
    point.id = i;
    points.push_back(point);
  }
  return;
}


void RansacLines::input_points(vector<config::LaserPoint> &points){
  config::LaserPoint sum_point;
  for(size_t i=0; i<points.size(); i++){
    sum_point.x = points[i].x;
    sum_point.y = points[i].y;
    sum_points.push_back(sum_point);
  }
}


vector<config::LaserPoint> RansacLines::get_inlier(vector<config::LaserPoint> &divided_points){
  if(divided_points.size()==0){
    vector<config::LaserPoint> zero_points;
    return zero_points;
  }
  inlier_points.clear();
  vector<config::LaserPoint> points_ = divided_points;
  int best_inlier_num=0;
  double best_diff_y=0;
  double best_diff_x=0;
  double best_diff_xy=0;
  // ランダムサンプリング
  for (int i = 0; i < trial_num_; i++) {
    // ランダムに2点を選択する
    int p1_idx = (int)(rand() / (RAND_MAX + 1.0) * points_.size());
    int p2_idx = (int)(rand() / (RAND_MAX + 1.0) * points_.size());
    const auto& p1 = points_[p1_idx];
    const auto& p2 = points_[p2_idx];
    // 直線の係数を計算する
    double diff_y = p2.y - p1.y;
    double diff_x = p1.x - p2.x;
    double diff_xy = p1.y * p2.x - p2.y * p1.x;
    detect_length = sqrt(diff_x*diff_x + diff_y*diff_y);
    if(detect_length > distance_threshold*1.1) continue;
    // インライア数を計算する
    int inlier_num = 0;
    for (const auto& point : points_) {
      double dist = std::abs(diff_y * point.x + diff_x * point.y + diff_xy) / std::sqrt(diff_y * diff_y + diff_x * diff_x);
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
      best_c = p2.y - best_a*p2.x;
    }
  }
  for(size_t i=0; i<best_inlier_num; i++){
    double dist = std::abs(best_diff_y * points_[i].x + best_diff_x * points_[i].y + best_diff_xy) / std::sqrt(best_diff_y * best_diff_y + best_diff_x * best_diff_x);
    if (dist < inlier_dist_threshold_) {
      config::LaserPoint temp_point;
      temp_point.x = points_[i].x;
      temp_point.y = points_[i].y;
      inlier_points.push_back(temp_point);
    }
  }
  return inlier_points;
}

void RansacLines::clear_points(vector<config::LaserPoint> &points, int size_threshold, int angle_threshold_min, int angle_threshold_max){
  if(points.size()<size_threshold || abs(radToDeg(atan(best_a))) > angle_threshold_max || abs(radToDeg(atan(best_a))) < angle_threshold_min){
    best_a=0;
    points.clear();
  }
}

vector<config::LaserPoint> RansacLines::get_filtered_rafter_right(){return rafter_right_points_;}
vector<config::LaserPoint> RansacLines::get_filtered_rafter_left(){return rafter_left_points_;}
vector<config::LaserPoint> RansacLines::get_filtered_rafter_back(){return rafter_back_points_;}
vector<config::LaserPoint> RansacLines::get_filtered_fence_centor_right(){return fence_centor_points_right_;}
vector<config::LaserPoint> RansacLines::get_filtered_fence_centor_left(){return fence_centor_points_left_;}
vector<config::LaserPoint> RansacLines::get_filtered_fence_front(){return fence_front_points_;}
vector<config::LaserPoint> RansacLines::get_filtered_fence_right(){return fence_right_points_;}
vector<config::LaserPoint> RansacLines::get_filtered_fence_left(){return fence_left_points_;}
vector<config::LaserPoint> RansacLines::get_sum(){return sum_points;}
