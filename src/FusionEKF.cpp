#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  /**
   * Basic initializations
   */
  is_initialized_ = false;
  previous_timestamp_ = 0;

  /**
   * Measurement-related matrices. These are not initialized
   * with the Kalman filter, but used during the measurement step when
   * they're needed.
   */
  // Laser measurement covariance
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  // Radar measurement covariance
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // Laser measurement matrix
  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Radar measurement matrix. This is not initialized to any value yet,
  // but will be done just before processing any radar measurement, by
  // using the "CalculateJacobian" function in `tools.cpp`.
  Hj_ = MatrixXd(3, 4);

  /**
   * State-related matrices.
   */
  // State covariance
  MatrixXd P_ = MatrixXd(4, 4);
  P_ << 1000,    0,    0,    0,
           0, 1000,    0,    0,
           0,    0, 1000,    0,
           0,    0,    0, 1000;
  ekf_.P_ = P_;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // First measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       * Convert radar from polar to cartesian coordinates and initialize state.
       */
      float m_rho = measurement_pack.raw_measurements_[0];
      float m_phi = measurement_pack.raw_measurements_[1];

      float m_x = m_rho * cos(m_phi);
      float m_y = m_rho * sin(m_phi);


      ekf_.x_[0] = m_x;
      ekf_.x_[1] = m_y;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
       * Initialize state.
       */
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   * We calculate the following things in the prediction step
   * 1. The time_delta, which is the time in seconds since the last measurement
   * 2. Using the time_delta, we generate a state transition matrix. This is set
   *    inside the Kalman filter before calling Predict.
   */
  long long int time_delta = measurement_pack.timestamp_ - previous_timestamp_;
  MatrixXd F_ = MatrixXd(4, 4);
  F_ << 1, 0, time_delta, 0,
        0, 1, 0         , time_delta,
        0, 0, 0         , 0,
        0, 0, 0         , 1;
  ekf_.F_ = F_;
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    Tools tools = Tools();

    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.Update(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
