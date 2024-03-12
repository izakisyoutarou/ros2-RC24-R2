#include "detection_interface/coordinate_transformation.hpp"

    Matrix3d coordinate_transformation::conversion(double px, double py, double depth){
        double h_angle = 0.0;
        double v_angle = 0.0;
        h_angle = (px - WIDTH / 2) * (HFOV / WIDTH);
        v_angle = (py - HEIGHT / 2) * (VFOV / HEIGHT);
        x = depth * tan(h_angle);
        y = depth * tan(v_angle);
        z = depth;
        
        Matrix3d camera_xyz;
        camera_xyz <<  x, 0.0, 0.0,
                       y, 0.0, 0.0, 
                       z, 0.0, 0.0;
        return camera_xyz;
    }

    Matrix3d coordinate_transformation::euler_angle(){ //Euler angle 
        Matrix3d Rx;
        Rx << 1.0,          0.0,           0.0,
              0.0, cos(theta_x), -sin(theta_x),
              0.0, sin(theta_x),  cos(theta_x);
        
        Matrix3d Ry;
        Ry <<  cos(theta_y), 0.0, sin(theta_y),
                        0.0, 1.0,          0.0,
              -sin(theta_y), 0.0, cos(theta_y);
        
        Matrix3d Rz;
        Rz << cos(theta_z), -sin(theta_z), 0.0,
              sin(theta_z),  cos(theta_z), 0.0,
                       0.0,           0.0, 1.0;
        
        Matrix3d R;
        R = Rx*Ry*Rz;
        return R;
    }

    Vector3d coordinate_transformation::Rx_Ry_Rz(double px, double py, double depth, Vector3d pose){ //variation
        Matrix3d before_xyz = conversion(px, py, depth);
        Matrix3d R = euler_angle();
        Matrix3d Rxyz;
        Rxyz = R * before_xyz;

        Matrix3d T;
        T <<  tx, 0.0, 0.0,
              ty, 0.0, 0.0, 
              tz, 0.0, 0.0;
        Matrix3d after_xyz;
        after_xyz = Rxyz * T;
        
        Vector3d XYZ;
        for(int i=0; i<3; i++) XYZ[i] = after_xyz(i,0);
        //cout << "x" << XYZ[0] << "y" << XYZ[1] << "z" << XYZ[2] << endl;
        
        XYZ = XYZ + pose;
        return XYZ;
    }

