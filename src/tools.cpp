#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  if (estimations.size() == 0) {
    cout << "Error - estimations size is 0" << endl;
    return rmse;
  }

  //  * the estimation vector size should equal ground truth vector size
  // ... your code here
  if (estimations.size() != ground_truth.size()) {
    cout << "Error - estimations size and ground truth size don't match" << endl;
    return rmse;
  }

  //accumulate squared residuals
  VectorXd residual(4);
  residual << 0,0,0,0;
  for(int i=0; i < estimations.size(); ++i){
    // ... your code here
    VectorXd diff(4);
    diff = (estimations[i] - ground_truth[i]);
    residual = residual.array() + (diff.array() * diff.array());
  }

  //calculate the mean
  VectorXd residual_mean(4);
  residual_mean = residual / estimations.size();

  //calculate the squared root
  rmse = residual_mean.array().sqrt();

  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  /**
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float mean_square = px * px + py * py;

  if (std::fabs(mean_square) < 0.001) {
    std::cout << "Error - Division by zero" << std::endl;
    return Hj;
  }

  float root_mean_square = sqrt(mean_square);
  float cubic_root_mean_square = pow(root_mean_square, 3);
  float product = vx * py - vy * px;
  float product2 = vy * px - vx * py;

  Hj << px / root_mean_square, py / root_mean_square, 0, 0,
          -py / mean_square, px / mean_square, 0, 0,
          py * product / cubic_root_mean_square, px * product2 / cubic_root_mean_square, px / root_mean_square, py /
                                                                                                                root_mean_square;

  return Hj;
}
