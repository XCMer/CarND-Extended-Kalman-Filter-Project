#include <cmath>
#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;

  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(2, 2) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Calculate Jacobian
  MatrixXd Hj(3,4);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);

  float mean_square = px*px + py*py;

  if (std::fabs(mean_square) < 0.001) {
    std::cout<<"Error - Division by zero"<<std::endl;
    return;
  }

  float root_mean_square = sqrt(mean_square);
  float cubic_root_mean_square = pow(root_mean_square, 3);
  float product = vx*py - vy*px;
  float product2 = vy*px - vx*py;

  Hj << px / root_mean_square, py / root_mean_square, 0, 0,
          -py/mean_square, px/mean_square, 0, 0,
          py*product/cubic_root_mean_square, px*product2/cubic_root_mean_square, px/root_mean_square, py/root_mean_square;

  // Update
  VectorXd y = z - Hj * x_;
  MatrixXd Hjt = Hj.transpose();
  MatrixXd S = Hj * P_ * Hjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Hjt * Si;

  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(2, 2) - K * Hj) * P_;
}
