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
  // Calculate the measurement delta
  VectorXd y = z - H_ * x_;

  // Calculate Kalman gain
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // Update state and its covariance
  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // Convert state to polar co-ordinates
  VectorXd hx_(3);
  float x_rms = sqrt(x_[0]*x_[0] + x_[1]*x_[1]);
  hx_ << x_rms, atan2(x_[1], x_[0]), (x_[0]*x_[2] + x_[1]*x_[3]) / x_rms;

  // Calculate the measurement delta
  VectorXd y = z - hx_;

  // Normalize phi to -pi to pi after subtraction
  // Reference: https://discussions.udacity.com/t/already-used-atan2-to-calculate-phi-in-hx-do-i-still-need-to-normalize-the-phi-in-y/242332/4
  // We're basically doing atan(tan(y[1])). However, atan normalizes to between -pi and pi
  y[1] = atan2(sin(y[1]), cos(y[1]));

  // Calculate K
  MatrixXd Hjt = H_.transpose();
  MatrixXd S = H_ * P_ * Hjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Hjt * Si;

  // Update state and its covariance
  x_ = x_ + (K * y);
  P_ = (MatrixXd::Identity(4, 4) - K * H_) * P_;
}
