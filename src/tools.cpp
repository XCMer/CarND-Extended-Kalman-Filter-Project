#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd output(4);
  output << 0.1, 0.1, 0.1, 0.1;

  return output;
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
