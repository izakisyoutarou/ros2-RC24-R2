#pragma once
#include "detection_interface/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
using namespace std;
using namespace Eigen;

class coordinate_transformation {
    public:
        Vector3d Rx_Ry_Rz(double px, double py, double depth, Vector3d pose);
    private:
        Matrix3d conversion(double px, double py, double depth);
        Matrix3d euler_angle(Vector3d pose);
        double WIDTH = 1280.0;
        double HEIGHT = 720.0;
        double HFOV = 86.0;
        double VFOV = 57.0;
        double ball_r = 97.75;
        double x = 0.0 ;
        double y = 0.0 ;
        double z = 0.0 ;
        double z_angle = 0.0;
        double theta_x = -180.0; 
        double theta_y = 118.5; //Camera mounting position
        double theta_z = 270.0;
        double tx = 0.2335; //offset robot and camera
        double ty = -0.0845; //offset robot and camera
        double tz = 0.0; //offset robot and camera
};

