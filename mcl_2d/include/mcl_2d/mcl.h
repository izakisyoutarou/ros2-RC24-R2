#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <random>
#include <cmath>
#include "tool.h"

class mcl
{
  struct particle{
    Eigen::Matrix4f pose;
    float score;
    Eigen::Matrix4Xf scan; // Only for maximum probability particle.
  };

private:
  int m_sync_count;

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen; //Standard mersenne_twister_engine seeded with rd()

  float imageResolution;
  float mapCenterX;
  float mapCenterY;
  float odomCovariance[6];
  int numOfParticle;
  std::vector<particle> particles;
  particle maxProbParticle;
  cv::Mat gridMap; // Gridmap for showing
  cv::Mat gridMapCV; // Gridmap for use (gaussian-blurred)
  Eigen::Matrix4f tf_laser2robot;
  Eigen::Matrix4f odomBefore;
  float minOdomDistance;
  float minOdomAngle;
  int repropagateCountNeeded;

  bool isOdomInitialized;
  int predictionCounter;

  void initializeParticles();
  void prediction(Eigen::Matrix4f diffPose);
  void weightning(Eigen::Matrix4Xf laser);
  void resampling();
  void showInMap();


public:
  mcl();
  ~mcl();
  void updateData(Eigen::Matrix4f pose, Eigen::Matrix4Xf laser);
  double position_x, position_y, angle;
};
