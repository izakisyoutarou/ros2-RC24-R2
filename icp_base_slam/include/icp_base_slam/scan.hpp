#pragma once
#include <iostream>

struct Laser_point{
  double x;                // 位置x
  double y;                // 位置y
  double yaw;
};

struct Pose{
  double x;
  double y;
  double yaw;
  Pose():x(0),y(0),yaw(0){}
};
