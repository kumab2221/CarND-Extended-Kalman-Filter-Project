#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <algorithm>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_     = false;
  previous_timestamp_ =     0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_      = MatrixXd(3, 4);

  MatrixXd P(4, 4);
  MatrixXd F(4, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225,      0,
                   0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09,      0,    0,
                 0, 0.0009,    0,
                 0,      0, 0.09;

  // initialize measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
  
  // initializing state covariance matrix
  P << 1, 0,    0,    0,
       0, 1,    0,    0,
       0, 0, 1000,    0,
       0, 0,    0, 1000;

  // initialize transition matrix
  float dt = 0;
  F << 1, 0, dt,  0,
       0, 1,  0, dt,
       0, 0,  1,  0,
       0, 0,  0,  1;

  ekf_.P_ = P;
  ekf_.F_ = F;
  
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    auto x = VectorXd(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];

      x << std::max(rho*cos(phi), 0.0001),
           std::max(rho*sin(phi), 0.0001),
           0.f,
           0.f;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      x << measurement_pack.raw_measurements_(0),
           measurement_pack.raw_measurements_(1),
           0.f,
           0.f;
    }
    
    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.x_ = x;
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  // compute the time elapsed between the current and previous measurements
  // dt - expressed in seconds
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // TODO: YOUR CODE HERE
  float dt_2 = dt   * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // set the process covariance matrix Q
  constexpr float noise_ax = 9.;
  constexpr float noise_ay = 9.;

  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax,               0, dt_3/2*noise_ax,               0,
                            0, dt_4/4*noise_ay,               0, dt_3/2*noise_ay,
              dt_3/2*noise_ax,               0,   dt_2*noise_ax,               0,
                            0, dt_3/2*noise_ay,               0,   dt_2*noise_ay;

  ekf_.Predict();

  /**
   * Update
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    Hj_ = tools.CalculateJacobian(ekf_.x_);

    ekf_.H_ = Hj_;
  	ekf_.R_ = R_radar_;
    ekf_.UpdateEKF( measurement_pack.raw_measurements_ );
  } else {
    // TODO: Laser updates
    ekf_.H_ = H_laser_;
  	ekf_.R_ = R_laser_;
    ekf_.Update( measurement_pack.raw_measurements_ );
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
