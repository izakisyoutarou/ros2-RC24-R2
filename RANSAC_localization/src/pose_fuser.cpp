#include "RANSAC_localization/pose_fuser.hpp"


void PoseFuser::setup(const double laser_weight, const double odom_weight){
  laser_weight_ = laser_weight;
  odom_weight_ = odom_weight;
}

void PoseFuser::init(){
  current_points.clear();
  reference_points.clear();
}

Vector3d PoseFuser::fuse_pose(const Vector3d &ransac_estimated, const Vector3d &scan_odom_motion, const Vector3d &current_scan_odom, const double dt_scan, const vector<config::LaserPoint> &src_points, const vector<config::LaserPoint> &global_points){
  if(global_points.size()==0) return current_scan_odom;
  init();
  NormalVector normal_vector = find_correspondence(src_points, global_points, current_points, reference_points);
  Eigen::Matrix2d ransac_cov = calculate_ransac_covariance(ransac_estimated, current_points, reference_points, normal_vector, laser_weight_);
  Eigen::Matrix2d scan_odom_motion_cov = calculate_motion_covariance(scan_odom_motion, dt_scan, odom_weight_);  // オドメトリで得た移動量の共分散
  Eigen::Matrix2d rotate_scan_odom_motion_cov = rotate_covariance(ransac_estimated, scan_odom_motion_cov);
  Eigen::Vector2d mu1(ransac_estimated[0], ransac_estimated[1]);
  Eigen::Vector2d mu2(current_scan_odom[0], current_scan_odom[1]);
  Eigen::Vector2d mu;
  Eigen::Matrix2d fused_cov;               // センサ融合後の共分散
  fuse(mu1, ransac_cov, mu2, rotate_scan_odom_motion_cov, mu, fused_cov);  // 2つの正規分布の融合
  Vector3d estimated;
  estimated[0]=mu[0];
  estimated[1]=mu[1];
  estimated[2]=normalize_yaw(ransac_estimated[2]);
  return estimated;
}

NormalVector PoseFuser::find_correspondence(const vector<config::LaserPoint> &src_points, const vector<config::LaserPoint> &global_points, vector<LaserPoint> &current_points, vector<LaserPoint> &reference_points){
  LaserPoint global;
  LaserPoint current;
  NormalVector normal_vector;
  double sum_x=0.0;
  double sum_y=0.0;
  for(size_t i=0; i<global_points.size(); i++){
    current.x = src_points[i].x;
    current.y = src_points[i].y;
    global.x = global_points[i].x;
    global.y = global_points[i].y;
    LaserPoint closest_reference = find_closest_vertical_point(global);
    sum_x+=closest_reference.nx;
    sum_y+=closest_reference.ny;
    current_points.push_back(current);
    reference_points.push_back(closest_reference);
  }
  double L = sqrt(sum_x*sum_x + sum_y*sum_y);
  normal_vector.normalize_x = sum_x / L; // 平均（正規化）
  normal_vector.normalize_y = sum_y / L;
  return normal_vector;
}

LaserPoint PoseFuser::find_closest_vertical_point(LaserPoint global){
  LaserPoint closest;
  LaserPoint vertical_distance;
  double distance_min = 100.0;
  double x[4] = {map_point_x[0], map_point_x[1], map_point_x[2], distance_min};  //4つ目が最小距離にならないようにしている。
  for(int i=0; i<4; i++){
    vertical_distance.x = fabs(x[i] - global.x);
    vertical_distance.y = fabs(map_point_y[4] - global.y);
    if(vertical_distance.x < distance_min){
      distance_min = vertical_distance.x;
      closest.x = x[i];
      closest.y = global.y;
    }
    if(vertical_distance.y < distance_min){
      distance_min = vertical_distance.y;
      closest.x = global.x;
      closest.y = map_point_y[i];
    }
  }
  if(closest.x==map_point_x[0] || closest.x==map_point_x[1] || closest.x==map_point_x[2]){
    closest.nx=1.0;
    closest.ny=0.0;
  }
  else{
    closest.nx=0.0;
    closest.ny=1.0;
  }
  return closest;
}

Eigen::Matrix2d PoseFuser::calculate_ransac_covariance(const Vector3d &ransac_estimated, vector<LaserPoint> &current_points, vector<LaserPoint> &reference_points, NormalVector normal_vector, const double laser_weight_){
  double dd = 0.00001;  //数値微分の刻み
  double da = 0.00001;  //数値微分の刻み
  vector<double> Jx; //ヤコビ行列のxの列
  vector<double> Jy; //ヤコビ行列のyの列

  for(size_t i=0; i<current_points.size(); i++){
    double vertical_distance   = calculate_vertical_distance(current_points[i], reference_points[i], ransac_estimated[0],    ransac_estimated[1],    ransac_estimated[2], normal_vector);
    double vertical_distance_x = calculate_vertical_distance(current_points[i], reference_points[i], ransac_estimated[0]+dd, ransac_estimated[1],    ransac_estimated[2], normal_vector);
    double vertical_distance_y = calculate_vertical_distance(current_points[i], reference_points[i], ransac_estimated[0],    ransac_estimated[1]+dd, ransac_estimated[2], normal_vector);
    Jx.push_back((vertical_distance_x - vertical_distance) / dd);
    Jy.push_back((vertical_distance_y - vertical_distance) / dd);
  }
  // ヘッセ行列の近似J^TJの計算
  Eigen::Matrix2d hes = Eigen::Matrix2d::Zero(2,2);          // 近似ヘッセ行列。0で初期化
  for (size_t i=0; i<Jx.size(); i++) {
    hes(0,0) += Jx[i] * Jx[i];
    hes(0,1) += Jx[i] * Jy[i];
    hes(1,1) += Jy[i] * Jy[i];
  }
  // J^TJが対称行列であることを利用
  hes(1,0) = hes(0,1);
  if(round(hes(0,0)*hes(1,1)) == round(hes(0,1)*hes(1,0))){
    hes(0,0)+=1;
    hes(1,1)+=1;
  }
  return svdInverse(hes)*laser_weight_;
}

double PoseFuser::calculate_vertical_distance(const LaserPoint current, const LaserPoint reference, double x, double y, double yaw, NormalVector normal_vector){
  double x_ = cos(yaw)*current.x - sin(yaw)*current.y + x;                     // clpを推定位置で座標変換
  double y_ = sin(yaw)*current.x + cos(yaw)*current.y + y;
  return (x_-reference.x)*normal_vector.normalize_x + (y_-reference.y)*normal_vector.normalize_y;
}

Eigen::Matrix2d PoseFuser::calculate_motion_covariance(const Vector3d &scan_odom_motion, const double dt_scan, const double odom_weight_){
  double dis = sqrt(scan_odom_motion[0]*scan_odom_motion[0] + scan_odom_motion[1]*scan_odom_motion[1]);   // 移動距離
  double vt = dis/dt_scan;                    // 並進速度[m/s]
  double vthre = 0.02;                   // vtの下限値。同期ずれで0になる場合の対処
  if (vt < vthre)
    vt = vthre;
  double dx = vt;
  double dy = vt;
  Eigen::Matrix2d C1;
  C1.setZero();                          // 対角要素だけ入れる
  C1(0,0) = 0.001*dx*dx;                 // 並進成分x
  C1(1,1) = 0.001*dy*dy;                 // 並進成分y
  // スケール調整
  return odom_weight_*C1;
}

Eigen::Matrix2d PoseFuser::rotate_covariance(const Vector3d &ransac_estimated, Eigen::Matrix2d &scan_odom_motion_cov){
  double cs = cos(ransac_estimated[2]);            // poseの回転成分thによるcos
  double sn = sin(ransac_estimated[2]);
  Eigen::Matrix2d J;                            // 回転のヤコビ行列
  J << cs, -sn,
       sn,  cs;
  Eigen::Matrix2d JT = J.transpose();
  return J*scan_odom_motion_cov*JT;  // 回転変換
}

/////// ガウス分布の融合 ///////
// 2つの正規分布を融合する。muは平均、cvは共分散。
double PoseFuser::fuse(const Eigen::Vector2d &mu1, const Eigen::Matrix2d &cv1, const Eigen::Vector2d &mu2, const Eigen::Matrix2d &cv2, Eigen::Vector2d &mu, Eigen::Matrix2d &cv){
  // 共分散行列の融合
  Eigen::Matrix2d IC1 = svdInverse(cv1);
  Eigen::Matrix2d IC2 = svdInverse(cv2);
  Eigen::Matrix2d IC = IC1 + IC2;
  cv = svdInverse(IC);

  // 平均の融合
  Eigen::Vector2d nu1 = IC1*mu1;
  Eigen::Vector2d nu2 = IC2*mu2;
  Eigen::Vector2d nu3 = nu1 + nu2;
  mu = cv*nu3;
}


// SVDを用いた逆行列計算
Eigen::Matrix2d PoseFuser::svdInverse(const Matrix2d &A) {
  size_t m = A.rows();
  size_t n = A.cols();
  JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);
  MatrixXd eU = svd.matrixU();
  MatrixXd eV = svd.matrixV();
  VectorXd eS = svd.singularValues();
  MatrixXd M1(m, n);
  for (size_t i=0; i<n; i++) {
    for (size_t j=0; j<n; j++) {
      M1(i,j) = eU(j,i)/eS[i];
    }
  }
  Matrix2d IA;
  for (size_t i=0; i<n; i++) {
    for (size_t j=0; j<n; j++) {
      IA(i,j) = 0;
      for (size_t k=0; k<n; k++)
        IA(i,j) += eV(i,k)*M1(k,j);
    }
  }
  return(IA);
}
