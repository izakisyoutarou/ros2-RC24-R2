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
        double VFOV = 57.0;
        double ball_r = 95.0;
        double x = 0.0 ;
        double y = 0.0 ;
        double z = 0.0 ;
        double z_angle = 0.0;
        double theta_x = 0.0;
        double theta_y = 62.0;
        double theta_z = 0.0;
        double tx = 0.27737; //offset robot and camera
        double ty = -0.037; //offset robot and camera
        double tz = 0.644825;
        // const double angle_offset_coff[10] = {-3.217e0,1.175e0,-2.929e-1,5.894e-2,5.350e-3,-2.400e-2,2.363e-4,1.1701e-3,-2.132e-4,-7.684e-4}; //最小二乗法の係数
        const double angle_offset_coff[10] = {-4.003e0,1.241e0,-3.386e-1,7.096e-2,1.311e-2,-2.183e-2,7.461e-4,1.487e-3,-5.936e-5,-6.829e-4}; //最小二乗法の係数
        const double tanh_offset_coff[3] = {13.27,2.32,0.29};  //tanh近似の係数
};

