#include "RANSAC_localization/detect_circles.hpp"
DetectCircles::DetectCircles(){
  circles.push_back(circle_self_right);
  circles.push_back(circle_self_center);
  circles.push_back(circle_self_left);
  circles.push_back(circle_opponent_right);
  circles.push_back(circle_opponent_left);
};

void DetectCircles::init(){
  estimated_diff = Vector3d::Zero();
  for(int i=0; i<circles_datas.size(); ++i){
    circles_datas[i].points.clear();
  }
  circles_datas.clear();
  circles_datas.resize(circles.size());
}

Vector3d DetectCircles::calc_diff_pose(const vector<LaserPoint> &src_points){
  init();
  if(!detect_circles_flag) return estimated_diff;
  devide_points(src_points);
  double best_rate=0.0;
  Vector3d best_circle=Vector3d::Zero();
  for(size_t i=0; i<circles_datas.size(); i++){
    if(circles_datas[i].points.size() < 4 ) continue;
    Vector3d circle = get_best_circle(circles_datas[i]);
    if(best_rate < circles_datas[i].rate && circles_datas[i].rate > 0.6){
      best_circle = circle;
      best_rate = circles_datas[i].rate;
      estimated_diff = circles_datas[i].rate*(circles[i]-circle);
    }
  }
  return estimated_diff;
}

void DetectCircles::devide_points(const vector<LaserPoint> &src_points){
  for(size_t i=0; i<circles.size(); i++){
    create_voxel(circles_datas[i].points, circles[i], src_points);
  }
}

void DetectCircles::create_voxel(vector<LaserPoint> &points, const Vector3d &circle, const vector<LaserPoint> &src_points){
  const double half_voxel_size=0.5;
  for(size_t i=0; i<src_points.size(); i++){
    if(circle[0]-half_voxel_size < src_points[i].x && circle[0]+half_voxel_size > src_points[i].x && circle[1]-half_voxel_size < src_points[i].y && circle[1]+half_voxel_size > src_points[i].y){
      LaserPoint lp_;
      lp_=src_points[i];
      points.push_back(lp_);
    }
  }
}

Vector3d DetectCircles::get_best_circle(CirclesData &circles_data) {
  int max_iterations=100;
  mt19937 rand_engine(chrono::system_clock::now().time_since_epoch().count());
  uniform_int_distribution<int> rand_dist(0, circles_data.points.size() - 1);
  Vector3d best_circle;     // 最も適合する円を保存する変数
  int best_count = 0;     // 円周上に存在する点の最大数
  double best_error = numeric_limits<double>::max();   // 円と各点の距離の二乗和の最小値
  for (int i=0; i < max_iterations; ++i) {
    // 3つの点をランダムに選択
    int p1 = rand_dist(rand_engine);
    int p2 = rand_dist(rand_engine);
    int p3 = rand_dist(rand_engine);
    // 同じ点を選ばないようにする
    while (p2 == p1) p2 = rand_dist(rand_engine);
    while (p3 == p1 || p3 == p2) p3 = rand_dist(rand_engine);
    // 3つの点を通る円を求める
    double x1 = circles_data.points[p1].x, y1 = circles_data.points[p1].y;
    double x2 = circles_data.points[p2].x, y2 = circles_data.points[p2].y;
    double x3 = circles_data.points[p3].x, y3 = circles_data.points[p3].y;
    double diff_x_1 = x1 - x2, diff_y_1 = y1 - y2;
    double diff_x_2 = x1 - x3, diff_y_2 = y1 - y3;
    double pow_1 = (x1 * x1 - x2 * x2) + (y1 * y1 - y2 * y2);
    double pow_2 = (x1 * x1 - x3 * x3) + (y1 * y1 - y3 * y3);
    Vector3d circle;
    circle[0] = (diff_y_2 * pow_1 - diff_y_1 * pow_2) / (2 * diff_x_1 * diff_y_2 - 2 * diff_y_1 * diff_x_2);
    circle[1] = (diff_x_1 * pow_2 - diff_x_2 * pow_1) / (2 * diff_x_1 * diff_y_2 - 2 * diff_y_1 * diff_x_2);
    circle[2] = sqrt((circle[0] - x1) * (circle[0] - x1) + (circle[1] - y1) * (circle[1] - y1));
    const double esp=0.005;
    if(!(circle[2] >= type_1_r-esp && circle[2] <= type_1_r+esp)) continue;
    // 各点が円周上に存在するか調べる
    int inliers_num = 0;
    for (int j = 0; j < circles_data.points.size(); ++j) {
      double d = distance(circles_data.points[j].x, circles_data.points[j].y, circle[0], circle[1]);
      if (d >= type_1_r-esp && d <= type_1_r+esp) {
        ++inliers_num;
      }
    }
    // 最も適合する円を更新する
    if (inliers_num > best_count) {
      best_circle = circle;
      best_count = inliers_num;
    }
  }
  circles_data.rate = static_cast<double>(best_count) / circles_data.points.size();
  return best_circle;
}
