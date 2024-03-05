#pragma once
#include "ditaction_interface/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;

class coordinate_transformation {
    public:
        // Vector3d Rx_Ry_Rz(double px, double py, double depth, Vector3d pose);
        Vector3d Rx_Ry_Rz(double px, double py, Vector3d pose);
        
    private:
        // Matrix3d conversion(double px, double py, double depth);
        Matrix3d conversion(double px, double py);
        Matrix3d euler_angle();
        double depth; //camera kara
        double WIDTH = 640.0;
        double HEIGHT = 480.0;
        double HFOV = 69.4;
        double VFOV = 42.5;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double theta_x = 45.0; //Camera mounting position
        double theta_y = 180.0;
        double theta_z = -90.0;
        double tx = 0.0; //offset robot and camera
        double ty = 0.0; //offset robot and camera
        double tz = 0.0; //offset robot and camera
};
