#include "ransac_localization/detect_lines.hpp"

void DetectLines::setup(vector<LineData> &_lines, const double &voxel_size, const int &trial_num, const double &inlier_dist_threshold, const double &inlier_length_threshold){
  lines = _lines;
  voxel_size_ = voxel_size;
  trial_num_ = trial_num;
  inlier_dist_threshold_ = inlier_dist_threshold;
  points_num_threshold = inlier_length_threshold / voxel_size_;
}

void DetectLines::init(){
  estimated_diff = Vector3d::Zero();
  sum.clear();
  for (int i=0; i<lines.size(); ++i){
    lines[i].ransac_input.clear();
    lines[i].ransac_inliers.clear();
  }
}

void DetectLines::fuse_inliers(const vector<LaserPoint> &src_points, const Vector3d &laser_pose){
  init();
  devide_points(src_points);
  get_inliers();
  calc_estimated_diff();
}

void DetectLines::calc_estimated_diff(){
  //estimated_diff[2] = calc_diff_angle();
  for (size_t i=0; i<lines.size(); i++){
    if(lines[i].ransac_inliers.size()==0) continue;
    if(lines[i].axis=='x') estimated_diff[1] = lines[i].p_1.y - calc_average(i);
    else if(lines[i].axis=='y') estimated_diff[0] = lines[i].p_1.x - calc_average(i);
  }
}

double DetectLines::calc_average(const int &num){
  double sum=0.0;
  for(size_t i=0; i<lines[num].ransac_inliers.size(); i++){
    if(lines[num].axis=='x') sum += lines[num].ransac_inliers[i].y;
    else if(lines[num].axis=='y') sum += lines[num].ransac_inliers[i].x;
  }
  return sum/lines[num].ransac_inliers.size();
}

double DetectLines::calc_diff_angle(){
  double diff_angle=0.0;
  double best_diff_angle=100.0;
  int get_angle_count=0;
  for(size_t i=0; i<lines.size(); i++){
    //cout<<lines[i].ransac_inliers.size()<<" "<<1.0/voxel_size_<<endl;
    if(lines[i].ransac_inliers.size() < 1.0/voxel_size_) continue;
    get_angle_count++;
    if(lines[i].axis=='x') diff_angle = lines[i].estimate_angle;
    else if(lines[i].axis=='y'){
      if(lines[i].estimate_angle<0) diff_angle = lines[i].estimate_angle + M_PI/2;
      else diff_angle = lines[i].estimate_angle - M_PI/2;
    }
    if(diff_angle < best_diff_angle) best_diff_angle = diff_angle;
  }
  if(get_angle_count==0) return 0.0;
  return best_diff_angle;
}

void DetectLines::devide_points(const vector<LaserPoint> &src_points){
  for(size_t i=0; i<lines.size(); i++){
    for(size_t j=0; j<src_points.size(); j++){
      if(lines[i].p_1.x - distance_threshold < src_points[j].x &&
         lines[i].p_2.x + distance_threshold > src_points[j].x &&
         lines[i].p_1.y - distance_threshold < src_points[j].y &&
         lines[i].p_2.y + distance_threshold > src_points[j].y){
        LaserPoint point;
        point = src_points[j];
        lines[i].ransac_input.push_back(point);
      }
    }
  }
}

void DetectLines::get_inliers(){
  for(size_t i=0; i<lines.size(); i++){
    ransac_line(lines[i]);
    clear_points(lines[i]);
    input_points(lines[i]);
  }
}

void DetectLines::input_points(const LineData &line){
  LaserPoint sum_point;
  for(size_t i=0; i<line.ransac_inliers.size(); i++){
    sum_point.x = line.ransac_inliers[i].x;
    sum_point.y = line.ransac_inliers[i].y;
    sum.push_back(sum_point);
  }
}

void DetectLines::ransac_line(LineData &line){
  if(line.ransac_input.size()==0)return;
  int best_inlier_num=0;
  double best_diff_y=0.0;
  double best_diff_x=0.0;
  double best_diff_xy=0.0;
  double best_a = 0.0;
  double best_b = 0.0;
  double best_c = 0.0;
  // ランダムサンプリング
  for(int i=0; i<trial_num_; i++){
    // ランダムに2点を選択する
    const int p1_idx = static_cast<int>(rand() / (RAND_MAX + 1.0) * line.ransac_input.size());
    const int p2_idx = static_cast<int>(rand() / (RAND_MAX + 1.0) * line.ransac_input.size());
    // 直線の係数を計算する
    const double diff_y = line.ransac_input[p2_idx].y - line.ransac_input[p1_idx].y;
    const double diff_x = line.ransac_input[p1_idx].x - line.ransac_input[p2_idx].x;
    const double diff_xy = line.ransac_input[p1_idx].y * line.ransac_input[p2_idx].x - line.ransac_input[p2_idx].y * line.ransac_input[p1_idx].x;
    // インライア数を計算する
    int inlier_num = 0;
    for (const auto& point : line.ransac_input) {
      const double dist = abs(diff_y * point.x + diff_x * point.y + diff_xy) / sqrt(diff_y * diff_y + diff_x * diff_x); //点と直線の距離
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
      best_c = line.ransac_input[p2_idx].y - best_a*line.ransac_input[p2_idx].x;
    }
  }
  for(size_t i=0; i<best_inlier_num; i++){
    const double dist = abs(best_diff_y * line.ransac_input[i].x + best_diff_x * line.ransac_input[i].y + best_diff_xy) / sqrt(best_diff_y * best_diff_y + best_diff_x * best_diff_x);
    if (dist < inlier_dist_threshold_) {
      LaserPoint temp_point;
      temp_point.x = line.ransac_input[i].x;
      temp_point.y = line.ransac_input[i].y;
      line.ransac_inliers.push_back(temp_point);
    }
  }
  line.estimate_angle=atan(best_a);
}

void DetectLines::clear_points(LineData &line){
  double min_angle;
  double max_angle;
  if(line.axis=='x'){
    min_angle=0.0;
    max_angle=M_PI/4;
  }
  else if(line.axis=='y'){
    min_angle=M_PI/4;
    max_angle=M_PI/2;
  }
  //cout<<"l "<<line.ransac_inliers.size()<<" p "<<points_num_threshold<<" ang "<<fabs(line.estimate_angle)<<" i "<<min_angle<<" a "<<max_angle<<endl;
  if(line.ransac_inliers.size()<points_num_threshold || fabs(line.estimate_angle) < min_angle || fabs(line.estimate_angle) > max_angle) line.ransac_inliers.clear();
}

vector<LaserPoint> DetectLines::get_sum(){return sum;}
Vector3d DetectLines::get_estimated_diff(){return estimated_diff;}
