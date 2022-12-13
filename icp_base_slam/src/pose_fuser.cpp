#include "icp_base_slam/pose_fuser.hpp"

vector<const LaserPoint*> current_points;   //対応がとれた現在スキャンの点群
vector<const LaserPoint*> reference_points;   //対応がとれた参照スキャンの点群

void PoseFuser::fuse_pose(const Pose &ndt_estimated, const Pose &scan_odom_motion, const Pose &predict, const double dt_scan, const PclCloud::Ptr current_cloud, const Eigen::Matrix4d transformation_matrix, Pose &estimated, double laser_weight_, double odom_weight_){
  current_points.clear();
  reference_points.clear();
  NormalVector normal_vector = find_correspondence(current_cloud, transformation_matrix, current_points, reference_points);
  Eigen::Matrix2d ndt_cov;  //ndtの共分散行列
  double ratio = calculate_ndt_covariance(ndt_estimated, ndt_cov, transformation_matrix, current_points, reference_points, normal_vector, laser_weight_);
  Eigen::Matrix2d scan_odom_motion_cov;
  calculate_motion_covariance(scan_odom_motion, dt_scan, scan_odom_motion_cov, odom_weight_);  // オドメトリで得た移動量の共分散
  Eigen::Matrix2d rotate_scan_odom_motion_cov;
  rotate_covariance(ndt_estimated, scan_odom_motion_cov, rotate_scan_odom_motion_cov);
  Eigen::Vector2d mu1(ndt_estimated.x, ndt_estimated.y);
  Eigen::Vector2d mu2(predict.x, predict.y);
  Eigen::Vector2d mu;
  Eigen::Matrix2d fused_cov;               // センサ融合後の共分散
  fuse(mu1, ndt_cov, mu2, rotate_scan_odom_motion_cov, mu, fused_cov);  // 2つの正規分布の融合
  estimated.set_val(mu[0], mu[1], ndt_estimated.yaw);
}

NormalVector PoseFuser::find_correspondence(const PclCloud::Ptr current_cloud, const Eigen::Matrix4d transformation_matrix, vector<const LaserPoint*> &current_points, vector<const LaserPoint*> &reference_points){
  LaserPoint global;
  LaserPoint* current;
  NormalVector normal_vector;
  double sum_x=0.0;
  double sum_y=0.0;

  for(size_t i=0; i<current_cloud->points.size(); i++){
    current->x = current_cloud->points[i].x;
    current->y = current_cloud->points[i].y;
    global.x = transformation_matrix(0,0)*current->x + transformation_matrix(0,1)*current->y + transformation_matrix(0,3);
    global.y = transformation_matrix(1,0)*current->x + transformation_matrix(1,1)*current->y + transformation_matrix(1,3);
    LaserPoint* closest_reference = find_closest_vertical_point(global);
    sum_x+=closest_reference->nx;
    sum_y+=closest_reference->ny;
    current_points.push_back(current);
    reference_points.push_back(closest_reference);
  }
  double L = sqrt(sum_x*sum_x + sum_y*sum_y);
  normal_vector.normalize_x = sum_x / L; // 平均（正規化）
  normal_vector.normalize_y = sum_y / L;
  return normal_vector;
}

LaserPoint* PoseFuser::find_closest_vertical_point(LaserPoint global){
  LaserPoint* closest;
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
      closest->x = x[i];
      closest->y = global.y;
    }
    if(vertical_distance.y < distance_min){
      distance_min = vertical_distance.y;
      closest->x = global.x;
      closest->y = y[i];
    }
  }
  if(closest->x==back_rafter_x || closest->x==fence_x || closest->x==front_rafter_x){
    closest->nx=1.0;
    closest->ny=0.0;
  }
  else{
    closest->nx=0.0;
    closest->ny=1.0;
  }
  return closest;
}

LaserPoint* PoseFuser::find_closest_point_in_voxel(PclCloud map_cloud, LaserPoint global, LaserPoint current){
  double distance_min = 12.0;
  LaserPoint cell_min;
  LaserPoint cell_max;
  LaserPoint* closest;
  double cell_radius = 0.05;
  cell_min.x = global.x - cell_radius;
  cell_min.y = global.y - cell_radius;
  cell_max.x = global.x + cell_radius;
  cell_max.y = global.y + cell_radius;
  for(size_t i=0; i<map_cloud.points.size(); i++){
    if(cell_min.x<map_cloud.points[i].x && cell_max.x>map_cloud.points[i].x && cell_min.y<map_cloud.points[i].y && cell_max.y>map_cloud.points[i].y){
      double distance = (map_cloud.points[i].x - global.x)*(map_cloud.points[i].x - global.x) + (map_cloud.points[i].y - global.y)*(map_cloud.points[i].y - global.y);
      if(distance<distance_min){
        distance_min = distance;
        closest->x = map_cloud.points[i].x;
        closest->y = map_cloud.points[i].y;
      }
    }
  }
  return closest;
}

double PoseFuser::calculate_ndt_covariance(const Pose &ndt_estimated, Eigen::Matrix2d &ndt_cov, const Eigen::Matrix4d transformation_matrix, vector<const LaserPoint*> &current_points, vector<const LaserPoint*> &reference_points, NormalVector normal_vector, double laser_weight_){
  double dd = 0.00001;  //数値微分の刻み
  double da = 0.00001;  //数値微分の刻み
  vector<double> Jx; //ヤコビ行列のxの列
  vector<double> Jy; //ヤコビ行列のyの列

  for(size_t i=0; i<current_points.size(); i++){
    double vertical_distance   = calculate_vertical_distance(current_points[i], reference_points[i], ndt_estimated.x,    ndt_estimated.y,    ndt_estimated.yaw, normal_vector);
    double vertical_distance_x = calculate_vertical_distance(current_points[i], reference_points[i], ndt_estimated.x+dd, ndt_estimated.y,    ndt_estimated.yaw, normal_vector);
    double vertical_distance_y = calculate_vertical_distance(current_points[i], reference_points[i], ndt_estimated.x,    ndt_estimated.y+dd, ndt_estimated.yaw, normal_vector);
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
  //逆行列がinfになることを防ぐ
  if(round(hes(0,0)*hes(1,1)) == round(hes(0,1)*hes(1,0))) hes(0,0)+=1;
  ndt_cov = svdInverse(hes);
  // ndt_cov = hes.inverse();
  // double vals[2], vec1[2], vec2[2];
  // double ratio = calEigen(ndt_cov, vals, vec1, vec2);            // 固有値計算して、退化具合を調べる
  double ratio=0.0;
  ndt_cov *= laser_weight_;
  return(ratio);
}

double PoseFuser::calculate_vertical_distance(const LaserPoint* current, const LaserPoint* reference, double x, double y, double yaw, NormalVector normal_vector){
  double x_ = cos(yaw)*current->x - sin(yaw)*current->y + x;                     // clpを推定位置で座標変換
  double y_ = sin(yaw)*current->x + cos(yaw)*current->y + y;
  return (x_-reference->x)*normal_vector.normalize_x + (y_-reference->y)*normal_vector.normalize_y;
}

void PoseFuser::calculate_motion_covariance(const Pose &scan_odom_motion, const double dt_scan, Eigen::Matrix2d &scan_odom_motion_cov, double odom_weight_){
  double dis = sqrt(scan_odom_motion.x*scan_odom_motion.x + scan_odom_motion.y*scan_odom_motion.y);   // 移動距離
  double vt = dis/dt_scan;                    // 並進速度[m/s]
  double vthre = 0.02;                   // vtの下限値。同期ずれで0になる場合の対処
  if (vt < vthre)
    vt = vthre;
  double dx = vt;
  double dy = vt;
  Eigen::Matrix2d C1;
  C1.setZero();                          // 対角要素だけ入れる
  C1(0,0) = 0.001*dx*dx;                 // 並進成分x
  C1(1,1) = 0.005*dy*dy;                 // 並進成分y
  // スケール調整
  scan_odom_motion_cov = odom_weight_*C1;
  // 確認用
  // double vals[2], vec1[2], vec2[2];
  // calEigen(scan_odom_motion_cov, vals, vec1, vec2);
}

void PoseFuser::rotate_covariance(const Pose &ndt_estimated, Eigen::Matrix2d &scan_odom_motion_cov, Eigen::Matrix2d &rotate_scan_odom_motion_cov){
  double cs = cos(ndt_estimated.yaw);            // poseの回転成分thによるcos
  double sn = sin(ndt_estimated.yaw);
  Eigen::Matrix2d J;                            // 回転のヤコビ行列
  J << cs, -sn,
       sn,  cs;
  Eigen::Matrix2d JT = J.transpose();
  rotate_scan_odom_motion_cov = J*scan_odom_motion_cov*JT;  // 回転変換
}

/////// ガウス分布の融合 ///////
// 2つの正規分布を融合する。muは平均、cvは共分散。
double PoseFuser::fuse(const Eigen::Vector2d &mu1, const Eigen::Matrix2d &cv1,  const Eigen::Vector2d &mu2, const Eigen::Matrix2d &cv2, Eigen::Vector2d &mu, Eigen::Matrix2d &cv) {
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

  // // 係数部の計算
  // Eigen::Vector2d W1 = IC1*mu1;
  // Eigen::Vector2d W2 = IC2*mu2;
  // Eigen::Vector2d W = IC*mu;
  // double A1 = mu1.dot(W1);
  // double A2 = mu2.dot(W2);
  // double A = mu.dot(W);
  // double K = A1+A2-A;
/*
  //printf("cv1: det=%g\n", cv1.determinant());
  printMatrix(cv1);
  //printf("cv2: det=%g\n", cv2.determinant());
  printMatrix(cv2);
  //printf("cv: det=%g\n", cv.determinant());
  printMatrix(cv);
*/

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
//    if (eS[i] < 1.0E-10)                   // AがSingularかどうかのチェック。今回はしない
//      return;
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
// 共分散行列covの並進成分だけを固有値分解し、固有値をvalsに、固有ベクトルをvec1とvec2に入れる。
double PoseFuser::calEigen(const Eigen::Matrix3d &cov, double *vals, double *vec1, double *vec2) {
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
void PoseFuser::calEigen2D( double (*mat)[2], double *vals, double *vec1, double *vec2) {
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
void PoseFuser::print2x2Matrix (const Eigen::Matrix2d & matrix){
  printf("| %3.3f %3.3f |\n", matrix(0,0), matrix(0,1));
  printf("| %3.3f %3.3f |\n", matrix(1,0), matrix(1,1));
}
