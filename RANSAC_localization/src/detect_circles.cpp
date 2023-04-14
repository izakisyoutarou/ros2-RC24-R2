#include "RANSAC_localization/detect_circles.hpp"

void DetectCircles::init(){
  // estimated_diff = Vector3d::Zero();
  for (int i=0; i<circles_points.size(); ++i){
    circles_points[i].clear();
  }
  circles_points.clear();
  circles_points.resize(5);
}

void DetectCircles::calc_pose(const vector<LaserPoint> &src_points){
  init();
  devide_points(src_points);
  for(size_t i=0; i<circles_points.size(); i++){
    if(circles_points[i].size()<=5) continue;
    Circle circle = get_inliers(circles_points[i]);
    cout << "rate> " << circle.rate << endl;
  }
}

void DetectCircles::devide_points(const vector<LaserPoint> &src_points){
  for(size_t i=0; i<src_points.size(); i++){
    create_voxel(src_points[i], circles_points[0], circle_self_right);
    create_voxel(src_points[i], circles_points[1], circle_self_center);
    create_voxel(src_points[i], circles_points[2], circle_self_left);
    create_voxel(src_points[i], circles_points[3], circle_opponent_right);
    create_voxel(src_points[i], circles_points[4], circle_opponent_left);
  }
}

void DetectCircles::create_voxel(const LaserPoint &lp, vector<LaserPoint> &points, const Circle &circle){
  const double voxel_size=0.5;
  if(circle.x-voxel_size < lp.x && circle.x+voxel_size > lp.x && circle.y-voxel_size < lp.y && circle.y+voxel_size > lp.y){
    LaserPoint lp_;
    lp_=lp;
    points.push_back(lp_);
  }
  return;
}

Circle DetectCircles::get_inliers(vector<LaserPoint> &points) {
  int max_iterations=100;
  mt19937 rand_engine(chrono::system_clock::now().time_since_epoch().count());
  uniform_int_distribution<int> rand_dist(0, points.size() - 1);
  Circle best_circle;     // 最も適合する円を保存する変数
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
    double x1 = points[p1].x, y1 = points[p1].y;
    double x2 = points[p2].x, y2 = points[p2].y;
    double x3 = points[p3].x, y3 = points[p3].y;
    double diff_x_1 = x1 - x2, diff_y_1 = y1 - y2;
    double diff_x_2 = x1 - x3, diff_y_2 = y1 - y3;
    double pow_1 = (x1 * x1 - x2 * x2) + (y1 * y1 - y2 * y2);
    double pow_2 = (x1 * x1 - x3 * x3) + (y1 * y1 - y3 * y3);
    Circle circle;
    circle.x = (diff_y_2 * pow_1 - diff_y_1 * pow_2) / (2 * diff_x_1 * diff_y_2 - 2 * diff_y_1 * diff_x_2);
    circle.y = (diff_x_1 * pow_2 - diff_x_2 * pow_1) / (2 * diff_x_1 * diff_y_2 - 2 * diff_y_1 * diff_x_2);
    circle.r = sqrt((circle.x - x1) * (circle.x - x1) + (circle.y - y1) * (circle.y - y1));
    const double esp=0.05;
    if(circle.r >= R-esp || circle.r <= R+esp) continue;
    // 各点が円周上に存在するか調べる
    int inliers_num = 0;
    for (int j = 0; j < points.size(); ++j) {
      double d = distance(points[j], circle.x, circle.y);
      if (d >= R-esp || d <= R+esp) {
        ++inliers_num;
      }
    }
    // 最も適合する円を更新する
    if (inliers_num > best_count) {
      best_circle.x = circle.x;
      best_circle.y = circle.y;
      best_circle.r = circle.r;
      best_count = inliers_num;
    }
  }
  cout << "best_count> " << best_count << endl;
  cout << "points.size> " << points.size() << endl;
  best_circle.rate = static_cast<double>(best_count) / points.size();
  return best_circle;
}
