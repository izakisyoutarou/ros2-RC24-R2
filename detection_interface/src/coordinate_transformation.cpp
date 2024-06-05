#include "detection_interface/coordinate_transformation.hpp" 

    Matrix3d coordinate_transformation::conversion(double px, double py, double depth){
        double h_angle = 0.0;
        double v_angle = 0.0;
        double x_pixel_dis = px - WIDTH/2;
        double y_pixel_dis = py - HEIGHT/2;
        double z_pixel_dis = 0.0;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        double r = 0.0;     //極座標空間における距離(depth)
        double fai = 0.0;   //極座標空間における角度(天頂角)
        double theta = 0.0; //極座標空間における角度(方位角)

        // double x_normalized = -(px-WIDTH/2)/(WIDTH/2)*tan(HFOV/2);
        h_angle = -x_pixel_dis*(HFOV/WIDTH);
        v_angle = y_pixel_dis*(VFOV/HEIGHT);
        h_angle = h_angle*M_PI/180 + angle_offset(v_angle,h_angle);

        r = depth+ball_r;                             //極座標へ変換
        fai = h_angle;                                //極座標へ変換
        theta = M_PI_2+(theta_y+v_angle)*M_PI/180;    //極座標へ変換

        // cout << "r " << r << endl;

        // cout <<"h_angle" << h_angle*180/M_PI << " v_angle " << v_angle*180/M_PI <<  endl;
        // cout << "v_angle " << v_angle << " h_angle " << h_angle*180/M_PI << endl;
        cout << "h_angle" << h_angle*180/M_PI << endl;
    
        x = r* sin(theta) * cos(fai)*0.001; //メートル単位に変換
        y = r* sin(theta) * sin(fai)*0.001; //メートル単位に変換
        z = r* cos(theta) * 0.001;          //メートル単位に変換

        cout << "x" << x << " y " << y << " z " << z << endl;

        Matrix3d camera_xyz;
        camera_xyz << x,0.0,0.0,
                      y,0.0,0.0,
                      z,0.0,0.0;

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

        Matrix3d after_xyz;
        after_xyz = R_self*(before_xyz + T);
    
        // before_xyz = R_self*before_xyz; //Rxyzを使用しなくしたため

        // cout <<"before_xyz" << endl;
        // cout << before_xyz << endl;

        // cout <<"after_xyz" << endl;
        // cout << after_xyz << endl;
        
        // T = R_self*T;

        // Matrix3d after_xyz;
        // after_xyz = before_xyz + T; //同上

        Vector3d XYZ;
        for(int i=0; i<3; i++) XYZ[i] = after_xyz(i,0);
        XYZ = XYZ + pose;
        return XYZ;
    }

    double coordinate_transformation::angle_offset(double v_angle,double h_angle){
        double angle_offset=0.0;
        double a = angle_offset_coff[0];
        double b = angle_offset_coff[1];
        double c = angle_offset_coff[2];
        double d = angle_offset_coff[3];
        double e = angle_offset_coff[4];
        double f = angle_offset_coff[5];
        // double g = angle_offset_coff[6];
        // double h = angle_offset_coff[7];
        // double i = angle_offset_coff[8];
        // double j = angle_offset_coff[9];
        double x = h_angle;
        double y = v_angle;
        // angle_offset = a + b*x + c*y + d*x*y + e*x*x + f*y*y + g*x*x*y + h*x*y*y + i*x*x*x+j*y*y*y;
        // angle_offset = a*x*x + b*y*y + c*x*y + d*x + e*y + f;   
        // angle_offset = tanh_offset_coff[0]*tanh(tanh_offset_coff[1]*h_angle*M_PI/180) + tanh_offset_coff[2]*v_angle*sign(h_angle);//tanh関数を用いた近似
        // angle_offset = sin(h_angle*M_PI/180)*(sin_offset_coff[0]*v_angle+sin_offset_coff[1])+sin_offset_coff[2];    //sin関数を用いた近似
        angle_offset = sin_offset_test_coff[0]*sin(sin_offset_test_coff[1]*h_angle*M_PI/180)*abs(v_angle)+sin_offset_test_coff[2]*(sin_offset_test_coff[3]-v_angle)*h_angle+sin_offset_test_coff[4];  //パラメータが決まり次第試す
        // cout << "angle_offset: " << angle_offset << endl;
        return angle_offset*M_PI/180;
        // return 0;
    }

    double coordinate_transformation::sign(double h_angle){
        if(h_angle<0) return -1;
        else if(h_angle==0) return 0;
        else return 1;
    }