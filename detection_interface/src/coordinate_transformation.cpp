#include "detection_interface/coordinate_transformation.hpp"

    Matrix3d coordinate_transformation::conversion(double px, double py, double depth){
        // cout << "px" << px << "py" << py << "depth" << depth << endl;
        double h_angle = 0.0;
        double v_angle = 0.0;
        h_angle = -(px - WIDTH / 2) * (HFOV / WIDTH);
        v_angle = -(py - HEIGHT / 2) * (VFOV / HEIGHT);

        // cout << "h " << h_angle << "  v " << v_angle << endl;
        
        x = depth* tan( h_angle * M_PI/180);
        y = depth* tan( v_angle * M_PI/180);
        z = depth;
        
        // cout << "x " << x << "  y " << y << "  z " << z << endl;
        
        Matrix3d camera_xyz;
        camera_xyz <<  x, 0.0, 0.0,
                       y, 0.0, 0.0, 
                       z, 0.0, 0.0;
        // cout << camera_xyz << endl;
        return camera_xyz;
    }

    Matrix3d coordinate_transformation::euler_angle(){ //Euler angle 
        Matrix3d Rx;
        Rx << 1.0,                     0.0,                      0.0,
              0.0, cos(theta_x * M_PI/180), -sin(theta_x * M_PI/180),
              0.0, sin(theta_x * M_PI/180),  cos(theta_x * M_PI/180);
        // cout << "Rx" << endl;
        // cout << Rx << endl;
        
        Matrix3d Ry;
        Ry <<  cos(theta_y * M_PI/180), 0.0, sin(theta_y * M_PI/180),
                                   0.0,            1.0,          0.0,
              -sin(theta_y * M_PI/180), 0.0, cos(theta_y * M_PI/180);
        // cout << "Ry" << endl;
        // cout << Ry << endl;

        Matrix3d Rz;
        Rz << cos(theta_z * M_PI/180), -sin(theta_z * M_PI/180), 0.0,
              sin(theta_z * M_PI/180),  cos(theta_z * M_PI/180), 0.0,
                                  0.0,                      0.0, 1.0;
        // cout << "Rz" << endl;
        // cout << Rz << endl;

        Matrix3d R;
        R = Rx*Ry*Rz;
        // cout << "R" << endl;
        // cout << R << endl;
        return R;
    }

    Vector3d coordinate_transformation::Rx_Ry_Rz(double px, double py, double depth, Vector3d pose){ //variation
        // cout << "x" << px << "y" << py << "depth" << depth << endl;
        Matrix3d before_xyz = conversion(px, py, depth);
        Matrix3d R = euler_angle();
        // cout << "before_xyz" << endl;
        // cout << before_xyz << endl;
        // cout << "R" << endl;
        // cout << R << endl;

        Matrix3d Rxyz;
        Rxyz = R * before_xyz;
        // cout << "Rxyz" << endl;
        // cout << Rxyz << endl;

        Matrix3d T;
        T <<  tx, 0.0, 0.0,
              ty, 0.0, 0.0, 
              tz, 0.0, 0.0;
        Matrix3d after_xyz;
        after_xyz = Rxyz + T;
        
        Vector3d XYZ;
        for(int i=0; i<3; i++) XYZ[i] = after_xyz(i,0);
        // cout << "x" << XYZ[0] << "y" << XYZ[1] << "z" << XYZ[2] << endl;
        
        XYZ = XYZ + pose;
        return XYZ;
    }

