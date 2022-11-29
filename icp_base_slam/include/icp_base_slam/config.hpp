#pragma once
#include <iostream>
#include <vector>
// #include "icp_base_slam/icp_base_slam.hpp"
using namespace std;

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
