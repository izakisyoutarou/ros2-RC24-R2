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
        h_angle = -x_pixel_dis*(HFOV/WIDTH)/**M_PI/180*/;
        v_angle = y_pixel_dis*(VFOV/HEIGHT)/**M_PI/180*/;
        h_angle = h_angle*M_PI/180 + angle_offset(v_angle,h_angle);
        // h_angle =atan(x_normalized/cos(theta_x*M_PI/180))+angle_offset(v_angle,h_angle);
        // h_angle = -x_pixel_dis*(HFOV/WIDTH)*M_PI/180;
        // cout << "test用" << h_angle*180/M_PI << " " << v_angle*180/M_PI << endl;
        cout << "h_angle " << h_angle*180/M_PI << "v_angle " << v_angle << endl;         

        r = depth+ball_r;                             //極座標へ変換
        fai = h_angle;                                //極座標へ変換
        theta = M_PI_2+theta_y*M_PI/180+v_angle;      //極座標へ変換
        
        // z_pixel_dis = sqrt(abs(r*r-(eliptic_coff[0]*x_pixel_dis*x_pixel_dis+eliptic_coff[1]*y_pixel_dis*y_pixel_dis)));
        // Matrix3d rot_y;
        // rot_y << cos(-theta_y*M_PI/180),0.0,sin(-theta*M_PI/180),
        //          0.0,1.0,0.0,
        //          -sin(-theta_y*M_PI/180),0.0,cos(-theta_y*M_PI/180);

        // Matrix3d pos_ball;
        // pos_ball << x_pixel_dis,0.0,0.0,
        //             y_pixel_dis,0.0,0.0,
        //             z_pixel_dis,0.0,0.0;
        // pos_ball = rot_y*pos_ball;
        // v_angle = atan2(pos_ball(1,0),pos_ball(2,0));
        // h_angle = atan2(pos_ball(0,0),pos_ball(2,0));

        // cout << "v_angle_test " << v_angle*180/M_PI << " h_angle " << h_angle*180/M_PI << endl;
        // v_angle = y_pixel_dis*(VFOV/HEIGHT)*M_PI/180;
        // h_angle = -x_pixel_dis*(HFOV/WIDTH)*M_PI/180;
        // cout << "v_angle " << v_angle*180/M_PI << " h_angle " << h_angle*180/M_PI << endl;
        
        
        // Matrix3d camera_xyz;
        // camera_xyz << x_pixel_dis,0.0,0.0,
        //               y_pixel_dis,0.0,0.0,
        //               pz,0.0,0.0;
        
        // Matrix3d Ry;
        // Ry << cos(-theta_y*M_PI/180),0.0,sin(-theta_y*M_PI/180),
        //                         0.0,1.0,0.0,
        //     -sin(-theta_y*M_PI/180),0.0,cos(-theta_y*M_PI/180);
        
        // camera_xyz = Ry*camera_xyz;

        // v_angle = y_pixel_dis*(VFOV/HEIGHT)*M_PI/180;
        // h_angle = -x_pixel_dis*(HFOV/WIDTH)*M_PI/180;

        // cout << "test" << atan2(camera_xyz(0,0),camera_xyz(2,0))*180/M_PI << endl;
        // cout <<"v_angle: " << v_angle*180/M_PI << " h_angle: " << h_angle*180/M_PI << endl;
        


        x = r* sin(theta) * cos(fai)*0.001; //メートル単位に変換
        y = r* sin(theta) * sin(fai)*0.001; //メートル単位に変換
        z = r* cos(theta) * 0.001;          //メートル単位に変換

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
    
        before_xyz = R_self*before_xyz; //Rxyzを使用しなくしたため
        
        T = R_self*T;

        Matrix3d after_xyz;
        after_xyz = before_xyz + T; //同上

        Vector3d XYZ;
        for(int i=0; i<3; i++) XYZ[i] = after_xyz(i,0);
        XYZ = XYZ + pose;        
        return XYZ;
    }

    double coordinate_transformation::angle_offset(double v_angle,double h_angle){
        double angle_offset=0.0;
        angle_offset = angle_offset_coff[0]*h_angle + angle_offset_coff[1]*v_angle + angle_offset_coff[2];
        // angle_offset = (angle_offset_coff[0]*v_angle+angle_offset_coff[1])*sin(h_angle);
        cout << "angle_offset: " << angle_offset << endl;
        return angle_offset*M_PI/180;
    }