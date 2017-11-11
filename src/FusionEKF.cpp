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
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1,    0,    0,    0,
             0,    1,    0,    0,
             0,    0, 1000,    0,
             0,    0,    0, 1000;

  // State transition
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  // Initialize tools
  tools = Tools();
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
      ekf_.x_[2] = 0;
      ekf_.x_[3] = 0;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
       * Initialize state.
       */
      ekf_.x_[0] = measurement_pack.raw_measurements_[0];
      ekf_.x_[1] = measurement_pack.raw_measurements_[1];
      ekf_.x_[2] = 0;
      ekf_.x_[3] = 0;
    }

    // Set the previous timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

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
   * 3. We also calculate the process covariance (Q)
   */
  float time_delta = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;

  // Update state transition matrix
  ekf_.F_(0, 2) = time_delta;
  ekf_.F_(1, 3) = time_delta;

  // Calculate process covariance
  float ax2 = 9;
  float ay2 = 9;
  float t2 = time_delta * time_delta;
  float t3 = time_delta * t2;
  float t4 = time_delta * t3;

  MatrixXd Q_ = MatrixXd(4, 4);
  Q_ << t4*ax2/4, 0       , t3*ax2/2, 0,
        0       , t4*ay2/4, 0       , t3*ay2/2,
        t3*ax2/2, 0       , t2*ax2  , 0,
        0       , t3*ay2/2, 0       , t2*ay2;
  ekf_.Q_ = Q_;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar Updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
