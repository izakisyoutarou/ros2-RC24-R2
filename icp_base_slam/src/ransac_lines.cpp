#include "icp_base_slam/ransac_lines.hpp"

vector<config::LaserPoint> sum_points;
vector<config::LaserPoint> rafter_right_points;
vector<config::LaserPoint> rafter_right_points_;
vector<config::LaserPoint> rafter_left_points;
vector<config::LaserPoint> rafter_left_points_;
vector<config::LaserPoint> rafter_back_points;
vector<config::LaserPoint> rafter_back_points_;
vector<config::LaserPoint> fence_centor_points;
vector<config::LaserPoint> fence_centor_points_;
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
double est_x=0.0;
double est_y=0.0;

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
  fence_centor_points.clear();
  fence_centor_points_.clear();
  fence_front_points.clear();
  fence_front_points_.clear();
  fence_right_points.clear();
  fence_right_points_.clear();
  fence_left_points.clear();
  fence_left_points_.clear();

  double distance_threshold=1.0;

  for(size_t i=0; i<src_cloud.points.size(); i++){
    if(map_point_x[0]-distance_threshold < src_cloud.points[i].x &&
       map_point_x[2]+distance_threshold > src_cloud.points[i].x &&
       map_point_y[0]-distance_threshold < src_cloud.points[i].y &&
       map_point_y[0]+distance_threshold > src_cloud.points[i].y){
      config::LaserPoint point1;
      point1.x = src_cloud.points[i].x;
      point1.y = src_cloud.points[i].y;
      rafter_right_points.push_back(point1);
    }
    if(map_point_x[0]-distance_threshold < src_cloud.points[i].x &&
       map_point_x[2]+distance_threshold > src_cloud.points[i].x &&
       map_point_y[3]-distance_threshold < src_cloud.points[i].y &&
       map_point_y[3]+distance_threshold > src_cloud.points[i].y){
      config::LaserPoint point2;
      point2.x = src_cloud.points[i].x;
      point2.y = src_cloud.points[i].y;
      rafter_left_points.push_back(point2);
    }
    if(map_point_x[0]-distance_threshold < src_cloud.points[i].x &&
       map_point_x[0]+distance_threshold > src_cloud.points[i].x &&
       map_point_y[0]-distance_threshold < src_cloud.points[i].y &&
       map_point_y[3]+distance_threshold > src_cloud.points[i].y){
      config::LaserPoint point3;
      point3.x = src_cloud.points[i].x;
      point3.y = src_cloud.points[i].y;
      rafter_back_points.push_back(point3);
    }
    if(map_point_x[2]-distance_threshold < src_cloud.points[i].x &&
       map_point_x[2]+distance_threshold > src_cloud.points[i].x &&
       map_point_y[0]-distance_threshold < src_cloud.points[i].y &&
       map_point_y[3]+distance_threshold > src_cloud.points[i].y){
      config::LaserPoint point4;
      point4.x = src_cloud.points[i].x;
      point4.y = src_cloud.points[i].y;
      fence_centor_points.push_back(point4);
    }
    if(map_point_x[1]-distance_threshold < src_cloud.points[i].x &&
       map_point_x[1]+distance_threshold > src_cloud.points[i].x &&
       map_point_y[1]-distance_threshold < src_cloud.points[i].y &&
       map_point_y[2]+distance_threshold > src_cloud.points[i].y){
      config::LaserPoint point5;
      point5.x = src_cloud.points[i].x;
      point5.y = src_cloud.points[i].y;
      fence_front_points.push_back(point5);
    }
    if(map_point_x[1]-distance_threshold < src_cloud.points[i].x &&
       map_point_x[2]+distance_threshold > src_cloud.points[i].x &&
       map_point_y[1]-distance_threshold < src_cloud.points[i].y &&
       map_point_y[1]+distance_threshold > src_cloud.points[i].y){
      config::LaserPoint point6;
      point6.x = src_cloud.points[i].x;
      point6.y = src_cloud.points[i].y;
      fence_right_points.push_back(point6);
    }
    if(map_point_x[1]-distance_threshold < src_cloud.points[i].x &&
       map_point_x[2]+distance_threshold > src_cloud.points[i].x &&
       map_point_y[2]-distance_threshold < src_cloud.points[i].y &&
       map_point_y[2]+distance_threshold > src_cloud.points[i].y){
      config::LaserPoint point7;
      point7.x = src_cloud.points[i].x;
      point7.y = src_cloud.points[i].y;
      fence_left_points.push_back(point7);
    }
  }

  if(rafter_right_points.size()>3){
    rafter_right_points_=get_inlier(rafter_right_points);
    input_points(rafter_right_points_);
  }
  if(rafter_left_points.size()>3){
    rafter_left_points_=get_inlier(rafter_left_points);
    input_points(rafter_left_points_);
  }
  if(rafter_back_points.size()>3){
    rafter_back_points_=get_inlier(rafter_back_points);
    input_points(rafter_back_points_);
  }
  if(fence_centor_points.size()>3){
    fence_centor_points_=get_inlier(fence_centor_points);
    input_points(fence_centor_points_);
  }
  if(fence_front_points.size()>3){
    fence_front_points_=get_inlier(fence_front_points);
    input_points(fence_front_points_);
  }
  if(fence_right_points.size()>3){
    fence_right_points_=get_inlier(fence_right_points);
    input_points(fence_right_points_);
  }
  if(fence_left_points.size()>3){
    fence_left_points_=get_inlier(fence_left_points);
    input_points(fence_left_points_);
  }
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
  inlier_points.clear();
  vector<config::LaserPoint> points_ = divided_points;
  // 最尤推定値
  int best_inlier_num = 0;
  // ランダムサンプリング
  for (int i = 0; i < trial_num_; i++) {
    // ランダムに2点を選択する
    int p1_idx = (int)(rand() / (RAND_MAX + 1.0) * points_.size());
    int p2_idx = (int)(rand() / (RAND_MAX + 1.0) * points_.size());
    const auto& p1 = points_[p1_idx];
    const auto& p2 = points_[p2_idx];
    // 直線の係数を計算する
    double a = p2.y - p1.y;
    double b = p1.x - p2.x;
    double c = p1.y * p2.x - p2.y * p1.x;
    // インライア数を計算する
    int inlier_num = 0;
    for (const auto& point : points_) {
      double dist = std::abs(a * point.x + b * point.y + c) / std::sqrt(a * a + b * b);
      if (dist < 0.1) {
        inlier_num++;
      }
    }
    // 最尤推定値を更新する
    if (inlier_num > best_inlier_num) {
      best_a = a;
      best_b = b;
      best_c = c;
      best_inlier_num = inlier_num;
    }
  }
  for(size_t i=0; i<best_inlier_num; i++){
    double dist = std::abs(best_a * points_[i].x + best_b * points_[i].y + best_c) / std::sqrt(best_a * best_a + best_b * best_b);
    if (dist < inlier_dist_threshold_) {
      config::LaserPoint temp_point;
      temp_point.x = points_[i].x;
      temp_point.y = points_[i].y;
      inlier_points.push_back(temp_point);
    }
  }
  return inlier_points;
}


vector<config::LaserPoint> RansacLines::get_filtered_rafter_right(){return rafter_right_points_;}
vector<config::LaserPoint> RansacLines::get_filtered_rafter_left(){return rafter_left_points_;}
vector<config::LaserPoint> RansacLines::get_filtered_rafter_back(){return rafter_back_points_;}
vector<config::LaserPoint> RansacLines::get_filtered_fence_centor(){return fence_centor_points_;}
vector<config::LaserPoint> RansacLines::get_filtered_fence_front(){return fence_front_points_;}
vector<config::LaserPoint> RansacLines::get_filtered_fence_right(){return fence_right_points_;}
vector<config::LaserPoint> RansacLines::get_filtered_fence_left(){return fence_left_points_;}
vector<config::LaserPoint> RansacLines::get_sum(){return sum_points;}
