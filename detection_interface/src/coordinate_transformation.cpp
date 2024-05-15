#include "detection_interface/coordinate_transformation.hpp" 

    Matrix3d coordinate_transformation::conversion(double px, double py, double depth){
        double h_angle = 0.0;
        double v_angle = 0.0;
        h_angle = (px - WIDTH / 2) * (HFOV / WIDTH);
        v_angle = (py - HEIGHT / 2) * (VFOV / HEIGHT);
        cout << "h " << h_angle << "  v " << v_angle << endl;
        
        x = depth* tan( h_angle * M_PI/180)*0.001;
        y = depth* tan( v_angle * M_PI/180)*0.001;
        z = (depth + ball_r)*0.001;
        
        Matrix3d camera_xyz;
        camera_xyz <<  x, 0.0, 0.0,
                       y, 0.0, 0.0, 
                       z, 0.0, 0.0;
        return camera_xyz;
    }

    Matrix3d coordinate_transformation::euler_angle(Vector3d pose){ //Euler angle 
        z_angle = pose[2]; 
        cout << z_angle << endl;
        Matrix3d Rx;
        Rx << 1.0,                     0.0,                      0.0,
              0.0, cos(theta_x * M_PI/180), -sin(theta_x * M_PI/180),
              0.0, sin(theta_x * M_PI/180),  cos(theta_x * M_PI/180);
        
        Matrix3d Ry;
        Ry <<  cos(theta_y * M_PI/180), 0.0, sin(theta_y * M_PI/180),
                                   0.0,            1.0,          0.0,
              -sin(theta_y * M_PI/180), 0.0, cos(theta_y * M_PI/180);

        Matrix3d Rz;
        Rz << cos(theta_z * M_PI/180), -sin(theta_z * M_PI/180), 0.0,
              sin(theta_z * M_PI/180),  cos(theta_z * M_PI/180), 0.0,
                                  0.0,                      0.0, 1.0;

        Matrix3d R_self;
        R_self << cos(z_angle), -sin(z_angle), 0.0,
                  sin(z_angle),  cos(z_angle), 0.0,
                            0.0,            0.0, 1.0;
        cout << "R_self" << endl;
        cout << R_self << endl;

        Matrix3d R;
        R = R_self*Rx*Ry*Rz;
        return R;
    }

    Vector3d coordinate_transformation::Rx_Ry_Rz(double px, double py, double depth, Vector3d pose){ //variation
        Matrix3d before_xyz = conversion(px, py, depth);
        Matrix3d R = euler_angle(pose);

        Matrix3d Rxyz;
        Rxyz = R * before_xyz;

        Matrix3d T;
        T <<  tx, 0.0, 0.0,
              ty, 0.0, 0.0, 
              tz, 0.0, 0.0;
        Matrix3d R_self;
        R_self << cos(z_angle), -sin(z_angle), 0.0,
                  sin(z_angle),  cos(z_angle), 0.0,
                            0.0,          0.0, 1.0;      
        T = R_self * T;

        Matrix3d after_xyz;
        after_xyz = Rxyz + T;
        
        Vector3d XYZ;
        for(int i=0; i<3; i++) XYZ[i] = after_xyz(i,0);
        XYZ = XYZ + pose;

        cout << XYZ << endl;
        return XYZ;
    }


