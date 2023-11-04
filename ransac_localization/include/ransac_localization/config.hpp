#pragma once
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace Eigen;

typedef Matrix<double, 6, 1> Vector6d;

struct LaserPoint{
  double x=0.0;
  double y=0.0;
};

struct LineData{
  LaserPoint p_1;
  LaserPoint p_2;
  char axis;
  vector<LaserPoint> ransac_input;
  vector<LaserPoint> ransac_inliers;
  double estimate_angle=0.0;

  void set_data(){
    check_axis();
    arrange();
  }

  void check_axis(){
    if(p_1.x==p_2.x) axis='y';
    else if(p_1.y==p_2.y) axis='x';
    else axis='e';
  }

  void arrange(){
    if(axis=='x'){
      if(p_1.x > p_2.x){
        double temp=p_1.x;
        p_1.x = p_2.x;
        p_2.x = temp;
      }
    }
    else if(axis=='y'){
      if(p_1.y > p_2.y){
        double temp=p_1.y;
        p_1.y = p_2.y;
        p_2.y = temp;
      }
    }
  }
};


constexpr double distance(const double x1, const double y1, const double x2, const double y2){
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

constexpr double get_angle(const double x1, const double y1, const double x2, const double y2){
  return atan2(y2 - y1, x2 - x1);
}

constexpr double radToDeg(const double rad){
  return rad*180/M_PI;
}

constexpr double normalize_yaw(double yaw){
  if (yaw < -M_PI) yaw += 2*M_PI;
  else if (yaw >= M_PI) yaw -= 2*M_PI;
  return yaw;
}

// 点群を並進・回転させる
constexpr LaserPoint rotate(LaserPoint point, double theta){
  LaserPoint p;
  p.x = point.x * cos(theta) - point.y * sin(theta);
  p.y = point.x * sin(theta) + point.y * cos(theta);
  return p;
}

inline vector<LaserPoint> transform(const vector<LaserPoint> &points, const Vector3d &pose) {
  vector<LaserPoint> transformed_points;
  for (const auto& point : points) {
    // 並進
    LaserPoint p = point;
    p.x += pose[0];
    p.y += pose[1];
    // 回転
    p = rotate(p, pose[2]);
    transformed_points.push_back(p);
  }
  return transformed_points;
}

inline int get_time_diff(chrono::system_clock::time_point &start){
  return chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now()-start).count();
}

inline Vector3d transform_sensor_position(const Vector6d& sensor_pos){
  Vector3d sensor_basic_pos(sensor_pos[0], sensor_pos[1], sensor_pos[2]);
  
  const double s_r = sin(sensor_pos[3]); //sinのroll角
  const double c_r = cos(sensor_pos[3]); //cosのroll角
  const double s_p = sin(sensor_pos[4]); //sinのpitch角
  const double c_p = cos(sensor_pos[4]); //cosのpitch角
  const double s_y = sin(sensor_pos[5]); //sinのyaw角
  const double c_y = cos(sensor_pos[5]); //cosのyaw角
  
  Matrix3d Rx, Ry, Rz, R;
  
  Rx << 1,   0,    0,
        0, c_r, -s_r,
        0, s_r,  c_r;
        
  Ry <<  c_p, 0, s_p,
           0, 1,   0,
        -s_p, 0, c_p;
        
  Rz << c_y, -s_y, 0,
        s_y,  c_y, 0,
          0,    0, 1;
        
  R = Rz * Ry * Rx;
  
  // センサの位置の変換
  Vector3d transformed_pos = R * sensor_basic_pos;
  
  return Vector3d(transformed_pos[0], transformed_pos[1], sensor_pos[5]); //x, y, yaw
}
