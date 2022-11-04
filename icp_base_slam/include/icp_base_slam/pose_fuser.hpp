#pragma once
#include "config.hpp"
#include "icp_base_slam/icp_base_slam.hpp"
#include <vector>
//lidarとオドメトリの推定値を融合する。(imuの融合器はその後作る)
class PoseFuser{
private:
  struct LaserPoint{
    double x;
    double y;
    LaserPoint():x(0.0),y(0.0){}

    void operator=(const LaserPoint &other) {
    x = other.x;
    y = other.y;
    }

    LaserPoint operator-(const LaserPoint &other){
      LaserPoint laser_point;
      laser_point.x = x - other.x;
      laser_point.y = y - other.y;
      return laser_point;
    }

    LaserPoint operator+(const LaserPoint &other){
      LaserPoint laser_point;
      laser_point.x = x + other.x;
      laser_point.y = y + other.y;
      return laser_point;
    }
  };


public:
  PoseFuser(){}

  const LaserPoint *get_data(const LaserPoint *data){
    return data;
  }

  void fuse_pose(const Pose &ndt_estimated, const Pose &scan_odom_motion, const Pose &predict, Pose &fused_pose, Eigen::Matrix3d &fused_cov, const double dt, const PclCloud::Ptr current_cloud, const Eigen::Matrix4d transformation_matrix){
    vector<LaserPoint> current_points;   //対応がとれた現在スキャンの点群
    vector<LaserPoint> reference_points;   //対応がとれた参照スキャンの点群
    find_correspondence(current_cloud, current_points, reference_points, transformation_matrix);
    Eigen::Matrix3d ndt_cov;  //ndtの共分散行列
    double ratio = calculate_ndt_covariance(ndt_estimated, current_points, reference_points, ndt_cov, transformation_matrix);

    // Eigen::Matrix3d scan_odom_motion_cov;
    // calculate_motion_covariance(scan_odom_motion, dt, scan_odom_motion_cov);  // オドメトリで得た移動量の共分散
    // Eigen::Matrix3d rotate_scan_odom_motion_cov;
    // rotate_covariance(ndt_estimated, scan_odom_motion_cov, rotate_scan_odom_motion_cov);
    // Eigen::Vector3d mu1(ndt_estimated.x, ndt_estimated.y, ndt_estimated.yaw);
    // Eigen::Vector3d mu2(predict.x, predict.y, predict.yaw);
    // Eigen::Vector3d mu;
    // fuse(mu1, ecov, mu2, mcov, mu, fusedCov);  // 2つの正規分布の融合
  }

  void find_correspondence(const PclCloud::Ptr current_cloud, vector<LaserPoint> &current_points, vector<LaserPoint> &reference_points, const Eigen::Matrix4d transformation_matrix){
    current_points.clear();
    reference_points.clear();
    LaserPoint global;
    LaserPoint current;
    LaserPoint closest_reference;

    for (size_t i=0; i<current_cloud->points.size(); i++){
      current.x = current_cloud->points[i].x;
      current.y = current_cloud->points[i].y;
      global.x = transformation_matrix(0,0)*current_cloud->points[i].x + transformation_matrix(0,1)*current_cloud->points[i].y + transformation_matrix(0,3);
      global.y = transformation_matrix(1,0)*current_cloud->points[i].x + transformation_matrix(1,1)*current_cloud->points[i].y + transformation_matrix(1,3);
      closest_reference = find_closest_vertical_point(global);
      current_points.push_back(current);
      reference_points.push_back(closest_reference);
    }
  }

  LaserPoint find_closest_vertical_point(LaserPoint global){
    LaserPoint closest;
    LaserPoint vertical_distance;
    double distance_min = 100.0;
    double back_rafter_x = 0.05-6.0;
    double fence_x = 2.0-6.0;
    double front_rafter_x = 5.9875-6.0;
    double right_rafter_y = 0.05-6.0;
    double right_fence_y = 2.0-6.0;
    double left_fence_y = 10.0-6.0;
    double left_rafter_y = 11.95-6.0;
    double x[4] = {back_rafter_x, fence_x, front_rafter_x, 100.0};  //4つ目が最小距離にならないようにしている。
    double y[4] = {right_rafter_y, right_fence_y, left_fence_y, left_rafter_y};
    for(int i=0; i<4; i++){
      vertical_distance.x = fabs(x[i] - global.x);
      vertical_distance.y = fabs(y[i] - global.y);
      if(vertical_distance.x < distance_min){
        distance_min = vertical_distance.x;
        closest.x = x[i];
        closest.y = global.y;
      }
      if(vertical_distance.y < distance_min){
        distance_min = vertical_distance.y;
        closest.y = y[i];
        closest.x = global.x;
      }
    }
    return closest;
  }

  LaserPoint find_closest_point_in_voxel(PclCloud map_cloud, LaserPoint global){
    double distance_min = 12.0;
    LaserPoint cell_min;
    LaserPoint cell_max;
    LaserPoint cell_radius;
    LaserPoint closest;
    cell_radius.x = cell_radius.y = 0.05;
    cell_min = global - cell_radius;
    cell_max = global + cell_radius;
    for(size_t i=0; i<map_cloud.points.size(); i++){
      if(cell_min.x<map_cloud.points[i].x && cell_max.x>map_cloud.points[i].x && cell_min.y<map_cloud.points[i].y && cell_max.y>map_cloud.points[i].y){
        double distance = (map_cloud.points[i].x - global.x)*(map_cloud.points[i].x - global.x) + (map_cloud.points[i].y - global.y)*(map_cloud.points[i].y - global.y);
        if(distance<distance_min){
          distance_min = distance;
          closest.x = map_cloud.points[i].x;
          closest.y = map_cloud.points[i].y;
        }
      }
    }
    return closest;
  }

  double calculate_ndt_covariance(const Pose &ndt_estimated, vector<LaserPoint> &current_points, vector<LaserPoint> &reference_points, Eigen::Matrix3d &ndt_cov, const Eigen::Matrix4d transformation_matrix){
    double dd = 0.00001;  //数値微分の刻み
    double da = 0.00001;  //数値微分の刻み
    vector<double> Jx; //ヤコビ行列のxの列
    vector<double> Jy; //ヤコビ行列のyの列
    vector<double> Jyaw; //ヤコビ行列のyawの列

    for(size_t i=0; i<current_points.size(); i++){
      double vertical_distance     = calculate_vertical_distance(current_points[i], reference_points[i], ndt_estimated.x,    ndt_estimated.y,    ndt_estimated.yaw);
      double vertical_distance_x   = calculate_vertical_distance(current_points[i], reference_points[i], ndt_estimated.x+dd, ndt_estimated.y,    ndt_estimated.yaw);
      double vertical_distance_y   = calculate_vertical_distance(current_points[i], reference_points[i], ndt_estimated.x,    ndt_estimated.y+dd, ndt_estimated.yaw);
      double vertical_distance_yaw = calculate_vertical_distance(current_points[i], reference_points[i], ndt_estimated.x,    ndt_estimated.y,    ndt_estimated.yaw+da);

      Jx.push_back  ((vertical_distance_x   - vertical_distance) / dd);
      Jy.push_back  ((vertical_distance_y   - vertical_distance) / dd);
      Jyaw.push_back((vertical_distance_yaw - vertical_distance) / da);
      printf("%f %f %f\n", (vertical_distance_x - vertical_distance) / dd, (vertical_distance_y - vertical_distance) / dd, (vertical_distance_yaw - vertical_distance) / da);
    }

    // ヘッセ行列の近似J^TJの計算
    Eigen::Matrix3d hes = Eigen::Matrix3d::Zero(3,3);          // 近似ヘッセ行列。0で初期化
    for (size_t i=0; i<Jx.size(); i++) {
      hes(0,0) += Jx[i]  *Jx[i];
      hes(0,1) += Jx[i]  *Jy[i];
      hes(0,2) += Jx[i]  *Jyaw[i];
      hes(1,1) += Jy[i]  *Jy[i];
      hes(1,2) += Jy[i]  *Jyaw[i];
      hes(2,2) += Jyaw[i]*Jyaw[i];
    }
    // J^TJが対称行列であることを利用
    hes(1,0) = hes(0,1);
    hes(2,0) = hes(0,2);
    hes(2,1) = hes(1,2);

    ndt_cov = svdInverse(hes);
    printf("ndt cov\n");
    printf("| %3.6f %3.6f %3.6f |\n", ndt_cov(0,0), ndt_cov(0,1), ndt_cov(0,2));
    printf("| %3.6f %3.6f %3.6f |\n", ndt_cov(1,0), ndt_cov(1,1), ndt_cov(1,2));
    printf("| %3.6f %3.6f %3.6f |\n", ndt_cov(2,0), ndt_cov(2,1), ndt_cov(2,2));
    double vals[2], vec1[2], vec2[2];
    double ratio = calEigen(ndt_cov, vals, vec1, vec2);            // 固有値計算して、退化具合を調べる

    // // 必要に応じて共分散行列のスケールを調整する
    // //  double kk = 1;          // 退化で極端にずれる場合
    double kk = 0.1;       // 通常
    ndt_cov *= kk;

    return(ratio);
  }

  double calculate_vertical_distance(const LaserPoint &current, const LaserPoint &reference, double x, double y, double yaw){
    double x_ = cos(yaw)*current.x - sin(yaw)*current.y + x;                     // clpを推定位置で座標変換
    double y_ = sin(yaw)*current.x + cos(yaw)*current.y + y;
    return (x_ - reference.x + y_ - reference.y);
  }


  // SVDを用いた逆行列計算
  Eigen::Matrix3d svdInverse(const Matrix3d &A) {
    size_t m = A.rows();
    size_t n = A.cols();

    JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

    MatrixXd eU = svd.matrixU();
    MatrixXd eV = svd.matrixV();
    VectorXd eS = svd.singularValues();

    MatrixXd M1(m, n);
    for (size_t i=0; i<n; i++) {
  //    if (eS[i] < 1.0E-10)                   // AがSingularかどうかのチェック。今回はしない
  //      return;
      for (size_t j=0; j<n; j++) {
        M1(i,j) = eU(j,i)/eS[i];
      }
    }

    Matrix3d IA;
    for (size_t i=0; i<n; i++) {
      for (size_t j=0; j<n; j++) {
        IA(i,j) = 0;
        for (size_t k=0; k<n; k++)
          IA(i,j) += eV(i,k)*M1(k,j);
      }
    }

    return(IA);
  }

  // 共分散行列covの並進成分だけを固有値分解し、固有値をvalsに、固有ベクトルをvec1とvec2に入れる。
  double calEigen(const Eigen::Matrix3d &cov, double *vals, double *vec1, double *vec2) {
    // 並進部分だけ取り出す
    double cv2[2][2];
    for (int i=0; i<2; i++)
      for (int j=0; j<2; j++)
        cv2[i][j] = cov(i,j);

    calEigen2D(cv2, vals, vec1, vec2);        // 固有値分解
    double ratio = vals[0]/vals[1];

    // 確認用
    // printf("Eigen: ratio=%g, val1=%g, val2=%g\n", ratio, vals[0], vals[1]);
    // printf("Eigen: vec1=(%g, %g), ang=%g\n", vec1[0], vec1[1], RAD2DEG(atan2(vec1[1], vec1[0])));

    return(ratio);
  }

  // 2次正方行列の固有値分解
  void calEigen2D( double (*mat)[2], double *vals, double *vec1, double *vec2) {
    double a = mat[0][0];
    double b = mat[0][1];
    double c = mat[1][0];
    double d = mat[1][1];

    double B = sqrt((a+d)*(a+d) - 4*(a*d-b*c));
    double x1 = ((a+d) + B)/2;
    double x2 = ((a+d) - B)/2;
    vals[0] = x1;                    // 固有値
    vals[1] = x2;

    double m00 = a-x1;
    double m01 = b;
    double L = sqrt(m00*m00 + m01*m01);
    vec1[0] = m01/L;                 // 固有ベクトル
    vec1[1] = -m00/L;

    m00 = a-x2;
    m01 = b;
    L = sqrt(m00*m00 + m01*m01);
    vec2[0] = m01/L;                 // 固有ベクトル
    vec2[1] = -m00/L;

  /*
    // 検算
    double ax1 = a*vec1[0] + b*vec1[1];
    double ax2 = x1*vec1[0];
    double ay1 = c*vec1[0] + d*vec1[1];
    double ay2 = x1*vec1[1];
    printf("ax1=%g, ax2=%g\n", ax1, ax2);
    printf("ay1=%g, ay2=%g\n", ay1, ay2);

    double prod = vec1[0]*vec2[0] + vec1[1]*vec2[1];
    printf("prod=%g\n", prod);
  */

  }

};




  // void calculate_motion_covariance(const Pose &scan_odom_motion, const double dt, Eigen::Matrix3d &scan_odom_motion_cov){
  //   double dis = sqrt(scan_odom_motion.x*scan_odom_motion.x + scan_odom_motion.y*scan_odom_motion.y);   // 移動距離
  //   double vt = dis/dt;                    // 並進速度[m/s]
  //   double wt = scan_odom_motion.yaw/dt;     // 角速度[rad/s]
  //   double vthre = 0.02;                   // vtの下限値。同期ずれで0になる場合の対処
  //   double wthre = 0.05;                   // wtの下限値

  //   if (vt < vthre)
  //     vt = vthre;
  //   if (wt < wthre)
  //     wt = wthre;

  //   double dx = vt;
  //   double dy = vt;
  //   double da = wt;

  //   Eigen::Matrix3d C1;
  //   C1.setZero();                          // 対角要素だけ入れる
  //   C1(0,0) = 0.001*dx*dx;                 // 並進成分x
  //   C1(1,1) = 0.005*dy*dy;                 // 並進成分y
  // //  C1(2,2) = 0.005*da*da;                 // 回転成分
  //   C1(2,2) = 0.05*da*da;                 // 回転成分

  //   // スケール調整
  // //  double kk = 100;                     // オドメトリのずれが大きい場合
  //   double kk = 1;                         // 通常
  //   scan_odom_motion_cov = kk*C1;

  //   // 確認用
  //   printf("calMotionCovarianceSimple\n");
  //   printf("vt=%g, wt=%g\n", vt, wt);
  //   double vals[2], vec1[2], vec2[2];
  //   calEigen(scan_odom_motion_cov, vals, vec1, vec2);
  //   printf("scan_odom_motion_cov : %g %g %g %g %g %g\n", scan_odom_motion_cov(0,0), scan_odom_motion_cov(0,1), scan_odom_motion_cov(0,2), scan_odom_motion_cov(1,1), scan_odom_motion_cov(1,2), scan_odom_motion_cov(2,2));
  // }

  // void rotate_covariance(const Pose &ndt_estimated, Eigen::Matrix3d &scan_odom_motion_cov, Eigen::Matrix3d &rotate_scan_odom_motion_cov){
  //   double cs = cos(ndt_estimated.yaw);            // poseの回転成分thによるcos
  //   double sn = sin(ndt_estimated.yaw);
  //   Eigen::Matrix3d J;                            // 回転のヤコビ行列
  //   J << cs, -sn, 0,
  //        sn, cs, 0,
  //        0, 0, 1;

  //   Eigen::Matrix3d JT = J.transpose();
  //   rotate_scan_odom_motion_cov = J*scan_odom_motion_cov*JT;  // 回転変換
  // }
