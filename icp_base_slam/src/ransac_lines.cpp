#include "icp_base_slam/ransac_lines.hpp"
vector<config::LaserPoint> sum_points;
vector<config::LaserPoint> RansacLines::fuse_inliers(PclCloud::Ptr &src_cloud){
  sum_points.clear();
  double distance_threshold=1.0;
  //右の垂木
  vector<config::LaserPoint> rafter_right_points;
  vector<config::LaserPoint> rafter_right_points_;
  double rafter_right_x_min=0.05-6;
  double rafter_right_x_max=5.9875-6;
  double rafter_right_y=0.05-6;
  //左の垂木
  vector<config::LaserPoint> rafter_left_points;
  vector<config::LaserPoint> rafter_left_points_;
  double rafter_left_x_min = rafter_right_x_min;
  double rafter_left_x_max = rafter_right_x_max;
  double rafter_left_y=11.95-6;
  //後ろの垂木
  vector<config::LaserPoint> rafter_back_points;
  vector<config::LaserPoint> rafter_back_points_;
  double rafter_back_x=0.05-6;
  double rafter_back_y_min=0.05-6;
  double rafter_back_y_max=11.95-6;
  //中心のフェンス
  vector<config::LaserPoint> fence_centor_points;
  vector<config::LaserPoint> fence_centor_points_;
  double fence_centor_x=5.9875-6;
  double fence_centor_y_min=0.05-6;
  double fence_centor_y_max=11.95-6;
  //正面のフェンス
  vector<config::LaserPoint> fence_front_points;
  vector<config::LaserPoint> fence_front_points_;
  double fence_front_x=2.0-6;
  double fence_front_y_min=2.0-6;
  double fence_front_y_max=10.0-6;
  //右のフェンス
  vector<config::LaserPoint> fence_right_points;
  vector<config::LaserPoint> fence_right_points_;
  double fence_right_x_min=2.0-6;
  double fence_right_x_max=5.9875-6;
  double fence_right_y=2.0-6;
  //左のフェンス
  vector<config::LaserPoint> fence_left_points;
  vector<config::LaserPoint> fence_left_points_;
  double fence_left_x_min=fence_right_x_min;
  double fence_left_x_max=fence_right_x_max;
  double fence_left_y=10.0-6;

  for(size_t i=0; i<src_cloud->points.size(); i++){
    if(rafter_right_x_min-distance_threshold < src_cloud->points[i].x &&
       rafter_right_x_max+distance_threshold > src_cloud->points[i].x &&
       rafter_right_y-distance_threshold < src_cloud->points[i].y &&
       rafter_right_y+distance_threshold > src_cloud->points[i].y){
      config::LaserPoint point1;
      point1.x = src_cloud->points[i].x;
      point1.y = src_cloud->points[i].y;
      rafter_right_points.push_back(point1);
    }
    if(rafter_left_x_min-distance_threshold < src_cloud->points[i].x &&
       rafter_left_x_max+distance_threshold > src_cloud->points[i].x &&
       rafter_left_y-distance_threshold < src_cloud->points[i].y &&
       rafter_left_y+distance_threshold > src_cloud->points[i].y){
      config::LaserPoint point2;
      point2.x = src_cloud->points[i].x;
      point2.y = src_cloud->points[i].y;
      rafter_left_points.push_back(point2);
    }
    if(rafter_back_x-distance_threshold < src_cloud->points[i].x &&
       rafter_back_x+distance_threshold > src_cloud->points[i].x &&
       rafter_back_y_min-distance_threshold < src_cloud->points[i].y &&
       rafter_back_y_max+distance_threshold > src_cloud->points[i].y){
      config::LaserPoint point3;
      point3.x = src_cloud->points[i].x;
      point3.y = src_cloud->points[i].y;
      rafter_back_points.push_back(point3);
    }
    if(fence_centor_x-distance_threshold < src_cloud->points[i].x &&
       fence_centor_x+distance_threshold > src_cloud->points[i].x &&
       fence_centor_y_min-distance_threshold < src_cloud->points[i].y &&
       fence_centor_y_max+distance_threshold > src_cloud->points[i].y){
      config::LaserPoint point4;
      point4.x = src_cloud->points[i].x;
      point4.y = src_cloud->points[i].y;
      fence_centor_points.push_back(point4);
    }
    if(fence_front_x-distance_threshold < src_cloud->points[i].x &&
       fence_front_x+distance_threshold > src_cloud->points[i].x &&
       fence_front_y_min-distance_threshold < src_cloud->points[i].y &&
       fence_front_y_max+distance_threshold > src_cloud->points[i].y){
      config::LaserPoint point5;
      point5.x = src_cloud->points[i].x;
      point5.y = src_cloud->points[i].y;
      fence_front_points.push_back(point5);
    }
    if(fence_right_x_min-distance_threshold < src_cloud->points[i].x &&
       fence_right_x_max+distance_threshold > src_cloud->points[i].x &&
       fence_right_y-distance_threshold < src_cloud->points[i].y &&
       fence_right_y+distance_threshold > src_cloud->points[i].y){
      config::LaserPoint point6;
      point6.x = src_cloud->points[i].x;
      point6.y = src_cloud->points[i].y;
      fence_right_points.push_back(point6);
    }
    if(fence_left_x_min-distance_threshold < src_cloud->points[i].x &&
       fence_left_x_max+distance_threshold > src_cloud->points[i].x &&
       fence_left_y-distance_threshold < src_cloud->points[i].y &&
       fence_left_y+distance_threshold > src_cloud->points[i].y){
      config::LaserPoint point7;
      point7.x = src_cloud->points[i].x;
      point7.y = src_cloud->points[i].y;
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

  return sum_points;
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
  vector<config::LaserPoint> points_ = divided_points;
  vector<config::LaserPoint> inlier_points;
  // 試行回数
  const int TRIAL_NUM = 100;
  // 最尤推定値
  double best_a = 0.0;
  double best_b = 0.0;
  double best_c = 0.0;
  int best_inlier_num = 0;
  // ランダムサンプリング
  for (int i = 0; i < TRIAL_NUM; i++) {
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
    if (dist < 0.1) {
      config::LaserPoint temp_point;
      temp_point.x = points_[i].x;
      temp_point.y = points_[i].y;
      inlier_points.push_back(temp_point);
    }
  }
  return inlier_points;
}
