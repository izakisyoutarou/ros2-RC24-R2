#pragma once
#include <iostream>
#include <vector>
#include <rclcpp/rclcpp.hpp>

using namespace std;

struct LaserPoint{
  double x=0.0;
  double y=0.0;
  double angle=0.0;
  double dist=0.0;
};

const double map_point_x[3] = {0.05-6., 2.-6., 5.9875-6.};
const double map_point_y[4] = {0.05-6., 2.-6, 10.-6., 11.95-6.};
constexpr double radToDeg(double rad){ return rad*180/M_PI; };
constexpr double normalize_yaw(double yaw){
  if (yaw < -M_PI) yaw += 2*M_PI;
  else if (yaw >= M_PI) yaw -= 2*M_PI;
  return yaw;
}
