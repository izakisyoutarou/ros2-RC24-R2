#include "detection_interface/coordinate_transformation.hpp" 

    Matrix3d coordinate_transformation::conversion(double px, double py, double depth){
        double h_angle = 0.0;
        double v_angle = 0.0;
        double r = 0.0;     //極座標空間における距離(depth)
        double fai = 0.0;   //極座標空間における角度(天頂角)
        double theta = 0.0; //極座標空間における角度(方位角)
        double x_normalized = (-(px-WIDTH/2)/(WIDTH/2))*tan(HFOV/2*M_PI/180);

        v_angle = (py - HEIGHT / 2) * (VFOV / HEIGHT) * M_PI/180;
        h_angle = atan(x_normalized/cos((theta_y)*M_PI/180))+angle_offset(py-HEIGHT/2);
        r = depth+ball_r;                             //極座標へ変換
        fai = h_angle;                                //極座標へ変換
        theta = M_PI_2+theta_y*M_PI/180+v_angle;      //極座標へ変換

        // cout <<"py: " << py << " h_angle: " << h_angle*180/M_PI << endl;
        
        x = r* sin(theta) * cos(fai)*0.001; //メートル単位に変換
        y = r* sin(theta) * sin(fai)*0.001; //メートル単位に変換
        z = r* cos(theta) * 0.001;          //メートル単位に変換
        
        Matrix3d camera_xyz;
        camera_xyz <<  x, 0.0, 0.0,
                       y, 0.0, 0.0,
                       z, 0.0, 0.0;

        return camera_xyz;
    }

    Matrix3d coordinate_transformation::euler_angle(Vector3d pose){ //Euler angle  
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

        Matrix3d R;
        R = Rx*Ry*Rz;
        return R;
    }

    Vector3d coordinate_transformation::Rx_Ry_Rz(double px, double py, double depth, Vector3d pose){ //variation
        Matrix3d before_xyz = conversion(px, py, depth);
        z_angle = pose[2];

        Matrix3d T;
        T <<  tx, 0.0, 0.0,
              ty, 0.0, 0.0, 
              tz, 0.0, 0.0;
        Matrix3d R_self;
        R_self << cos(z_angle), -sin(z_angle), 0.0,
                  sin(z_angle),  cos(z_angle), 0.0,
                            0.0,          0.0, 1.0;
    
        before_xyz = R_self*before_xyz; //Rxyzを使用しなくしたため
        
        T = R_self*T;

        Matrix3d after_xyz;
        after_xyz = before_xyz + T; //同上

        Vector3d XYZ;
        for(int i=0; i<3; i++) XYZ[i] = after_xyz(i,0);
        XYZ = XYZ + pose;        
        return XYZ;
    }

    double coordinate_transformation::angle_offset(int py){
        // cout <<"px-Heihgt: " << py << "offset: " << (angle_offset_coff[0]*py+angle_offset_coff[1])*M_PI/180 << endl;
        return (angle_offset_coff[0]*py+angle_offset_coff[1])*M_PI/180;
    }