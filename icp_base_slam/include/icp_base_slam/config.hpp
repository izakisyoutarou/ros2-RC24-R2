#pragma once
#include <iostream>
#include "icp_base_slam/icp_base_slam.hpp"
using namespace std;
using namespace Eigen;
using PointType = pcl::PointXYZ;
using PclCloud = pcl::PointCloud<PointType>;

struct Pose{
  double x;
  double y;
  double yaw;

  Pose():x(0.0),y(0.0),yaw(0.0){}

  void operator=(const Pose &other) {
    x   = other.x;
    y   = other.y;
    yaw = other.yaw;
  }

  Pose operator-(const Pose &other){
    Pose pose;
    pose.x   = x   - other.x;
    pose.y   = y   - other.y;
    pose.yaw = yaw - other.yaw;
    return pose;
  }

  Pose operator+(const Pose &other){
    Pose pose;
    pose.x   = x   + other.x;
    pose.y   = y   + other.y;
    pose.yaw = yaw + other.yaw;
    return pose;
  }
};

