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
        double angle_offset(double v_angle, double h_angle);
        double sign(double h_angle);
    private:
        Matrix3d conversion(double px, double py, double depth);
        Matrix3d euler_angle(Vector3d pose);
        double WIDTH = 1280;
        double HEIGHT = 720.0;
        double HFOV = 86.0;
        double VFOV = 58.0;
        double ball_r = 95.0;
        double x = 0.0 ;
        double y = 0.0 ;
        double z = 0.0 ;
        double z_angle = 0.0;
        double theta_x = 0.0;
        double theta_y = 30.0;
        double theta_z = 0.0;
        double tx = 0.27737; //offset robot and camera
        double ty = -0.037; //offset robot and camera
        double tz = 0.0;
        const double angle_offset_coff[3] = {0.4579,-0.12759,0.014997}; //最小二乗法の係数
        const double tanh_offset_coff[3] = {15.00,3.00,0.15};  //tanh近似の係数
};

