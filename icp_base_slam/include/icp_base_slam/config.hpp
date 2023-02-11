#pragma once
#include <iostream>
#include <vector>

using namespace std;

namespace config{
struct LaserPoint{
  double x=0.0;
  double y=0.0;
  double angle=0.0;
  double dist=0.0;
};
}

struct EstimatedLine{
  double angle=0.0;
  vector<config::LaserPoint> points;
};


struct Pose{
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

  // Pose():x(0.0),y(0.0),yaw(0.0){}
  Pose():Pose(0.0,0.0,0.0){}

  Pose(double x, double y, double yaw){
    this->x = x;
    this->y = y;
    this->yaw = yaw;
  }

  void set_val(double x, double y, double yaw) {
    this->x = x;
    this->y = y;
    this->yaw = yaw;
  }

  void set_init(){
    x = -5.5;
    y = 0.0;
    yaw = 0.0;
  }

  void operator=(const Pose &other) {
    x   = other.x;
    y   = other.y;
    yaw = other.yaw;
  }

  Pose operator-(const Pose &other){
    Pose pose;
    pose.x   = this->x   - other.x;
    pose.y   = this->y   - other.y;
    pose.yaw = this->yaw - other.yaw;
    return pose;
  }

  Pose operator+(const Pose &other){
    Pose pose;
    pose.x   = this->x   + other.x;
    pose.y   = this->y   + other.y;
    pose.yaw = this->yaw + other.yaw;
    return pose;
  }

  void operator+=(const Pose &other){
    x   += other.x;
    y   += other.y;
    yaw += other.yaw;
  }
};

const double map_point_x[3] = {0.05-6., 2.-6., 5.9875-6.};
const double map_point_y[4] = {0.05-6., 2.-6, 10.-6., 11.95-6.};
constexpr double radToDeg(double rad){ return rad*180/M_PI; };
