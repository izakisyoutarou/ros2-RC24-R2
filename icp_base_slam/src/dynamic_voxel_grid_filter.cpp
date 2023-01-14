#include "icp_base_slam/dynamic_voxel_grid_filter.hpp"

vector<config::LaserPoint> DynamicVoxelGridFilter::variable_voxel(vector<config::LaserPoint> &points){
  int voxel_num_x = 10;
  int voxel_num_y = 10;
  int voxel_index = -1;
  vector<config::LaserPoint> src_points = points;
  vector<vector<config::LaserPoint>> box(voxel_num_x*voxel_num_y);
  config::LaserPoint max;
  config::LaserPoint min;
  max = calc_max(src_points);
  min = calc_min(src_points);
  double diff_x = max.x - min.x;
  double diff_y = max.y - min.y;
  double resolution_x = diff_x/voxel_num_x;
  double resolution_y = diff_y/voxel_num_y;
  for(size_t j=0; j<voxel_num_x; j++){
    double voxel_size_threshold_x = j*resolution_x + min.x;
    for(size_t k=0; k<voxel_num_y; k++){
      voxel_index++;
      double voxel_size_threshold_y = k*resolution_y + min.y;
      for(size_t i=0; i<src_points.size(); i++){
        if(voxel_size_threshold_x<=src_points[i].x && src_points[i].x<voxel_size_threshold_x+resolution_x &&
           voxel_size_threshold_y<=src_points[i].y && src_points[i].y<voxel_size_threshold_y+resolution_y){
          config::LaserPoint point_;
          point_.x = src_points[i].x;
          point_.y = src_points[i].y;
          box[voxel_index].push_back(point_);
        }
      }
    }
  }
  vector<config::LaserPoint> centor;
  for(size_t i=0; i<=voxel_index; i++){
    vector<config::LaserPoint> p_vec;
    p_vec.assign(box[i].begin(), box[i].end());
    config::LaserPoint centor_p = calc_centroid(p_vec);
    if(isfinite(centor_p.x) && isfinite(centor_p.y)) centor.push_back(centor_p);
  }
  return centor;
}

config::LaserPoint DynamicVoxelGridFilter::calc_centroid(vector<config::LaserPoint> &points){
  config::LaserPoint centroid;
  // 重心を求める
  double total_x = 0;
  double total_y = 0;
  for (size_t i = 0; i < points.size(); i++){
    total_x += points[i].x;
    total_y += points[i].y;
    // printf("point x->%2.2f y->%2.2f\n", points[i].x, points[i].y);
  }
  centroid.x = total_x / points.size();
  centroid.y = total_y / points.size();
  // printf("x->%f y->%f\n", centroid.x, centroid.y);
  return centroid;
}

config::LaserPoint DynamicVoxelGridFilter::calc_min(vector<config::LaserPoint> &points){
  config::LaserPoint min = points[0];
  for(size_t i=0; i<points.size(); i++){
    if(points[i].x < min.x) min.x = points[i].x;
    else if(points[i].y < min.y) min.y = points[i].y;
  }
  return min;
}

config::LaserPoint DynamicVoxelGridFilter::calc_max(vector<config::LaserPoint> &points){
  config::LaserPoint max = points[0];
  for(size_t i=0; i<points.size(); i++){
    if(points[i].x > max.x) max.x = points[i].x;
    else if(points[i].y > max.y) max.y = points[i].y;
  }
  return max;
}
