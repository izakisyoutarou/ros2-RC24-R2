#include "detection_interface/coordinate_transformation.hpp" 

    Matrix3d coordinate_transformation::conversion(double px, double py, double depth){
        double h_angle = 0.0;
        double v_angle = 0.0;
        double r = 0.0;     //極座標空間における距離(depth)
        double fai = 0.0;   //極座標空間における角度(天頂角)
        double theta = 0.0; //極座標空間における角度(方位角)
        // h_angle = (px - WIDTH / 2) * (HFOV / WIDTH);
        // v_angle = (py - HEIGHT / 2) * (VFOV / HEIGHT);
        h_angle = -(px - WIDTH / 2) * (HFOV / WIDTH);
        v_angle = (py - HEIGHT / 2) * (VFOV / HEIGHT);
        r = depth;                                  //極座標へ変換
        fai = h_angle*M_PI/180;                              //極座標へ変換
        theta = M_PI_2+(theta_y+v_angle)*M_PI/180;    //極座標へ変換

        // cout << "h_angle: " << h_angle << "v_angle: " << v_angle << endl;
        
        // x = depth* tan( h_angle * M_PI/180)*0.001;
        // y = depth* tan( v_angle * M_PI/180)*0.001;
        // z = (depth + ball_r)*0.001;
        x = r* sin(theta) * cos(fai)*0.001; //メートル単位に変換
        y = r* sin(theta) * sin(fai)*0.001; //メートル単位に変換
        z = r* cos(theta) * 0.001;          //メートル単位に変換

        cout << "x: " << x << "y: " << y << "z: " << z <<endl;
                
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
        // R = R_self*Rz*Rx*Ry;
        // R = Rz*Rx*Ry;
        R = Rx*Ry*Rz;
        return R;
    }

    Vector3d coordinate_transformation::Rx_Ry_Rz(double px, double py, double depth, Vector3d pose){ //variation
        Matrix3d before_xyz = conversion(px, py, depth);
        z_angle = pose[2];

        // cout << "-before_xyz-"<<endl;
        // cout << before_xyz <<endl;
        // Matrix3d R = euler_angle(pose);  //カメラ座標に変換する際に、カメラの傾きを考慮して変換したため、self_poseの回転のみでよくなったため。

        // Matrix3d Rxyz;   //同上
        // Rxyz = R * before_xyz;   //同上

        // cout << "-Rxyz_before-"<<endl;
        // cout << Rxyz << endl;

        Matrix3d T;
        T <<  tx, 0.0, 0.0,
              ty, 0.0, 0.0, 
              tz, 0.0, 0.0;
        Matrix3d R_self;
        R_self << cos(z_angle), -sin(z_angle), 0.0,
                  sin(z_angle),  cos(z_angle), 0.0,
                            0.0,          0.0, 1.0;      
        // cout << "------------"<<endl;
        // cout << R_self<<endl;

        // Rxyz = Rxyz + T;
        // Rxyz = R_self*Rxyz;
        before_xyz = R_self*before_xyz; //Rxyzをコメントアウトしたため

        // cout << "-Rxyz_after-" << endl;
        // cout << Rxyz << endl;
        cout << "before_xyz" << endl;
        cout << before_xyz << endl;
        
        T = R_self*T;
        cout << "T" << endl;
        cout << T << endl;

        Matrix3d after_xyz;
        // after_xyz = Rxyz + T;
        after_xyz = before_xyz + T; //同上
        // after_xyz = Rxyz;
        // cout << "-after_xyz: " << after_xyz << endl;

        Vector3d XYZ;
        for(int i=0; i<3; i++) XYZ[i] = after_xyz(i,0);
        XYZ = XYZ + pose;

        // cout << XYZ << endl;
        // cout << "------------" << endl;
        
        return XYZ;
    }