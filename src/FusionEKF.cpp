#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>
#include <math.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

// Constructor.
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // The covariance matrices for laser and radar represent
  // uncertainty in sensor measurements

  //measurement covariance matrix - laser
  R_laser_ = MatrixXd(2, 2);
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  // Jacobian Matrix
  Hj_ = MatrixXd(3, 4);
  Hj_ <<  0, 0, 0, 0,
          0, 0, 0, 0,
          0, 0, 0, 0;
}

// Destructor.
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  // Initialization
  if (!is_initialized_) {

    // first measurement
    std::cout << "Initializing EKF" << endl;
    previous_timestamp_ = measurement_pack.timestamp_;

    // State Vector (px,py,vx,vy)
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 0, 0, 0, 0;

    // Transition matrix F_
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ <<  1, 0, 1, 0,
                0, 1, 0, 1,
                0, 0, 1, 0,
                0, 0, 0, 1;

    // State Covariance Matrix P
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<  0.15, 0, 0, 0,
                0, 0.15, 0, 0,
                0, 0, 0.15, 0,
                0, 0, 0, 0.15;

    // Inital Q Matrix
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) 
    {
      // Convert radar from polar to cartesian coordinates and initialize state.
      auto ro = measurement_pack.raw_measurements_[0];
      auto theta = measurement_pack.raw_measurements_[1];
      //auto rodot = measurement_pack.raw_measurements_[2];

      auto px = cos(theta) * ro;
      auto py = sin(theta) * ro;

      ekf_.x_ << px, py, 0.0, 0.0;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) 
    {
      auto px = measurement_pack.raw_measurements_[0];
      auto py = measurement_pack.raw_measurements_[1];

      ekf_.x_ << px, py, 0, 0;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  //////////// Prediction ///////////////////////////

  // Update the state transition matrix F according to the new elapsed time.
  // Time is measured in seconds.
  // Update the process noise covariance matrix.
  // Use noise_ax = 9 and noise_ay = 9 for your Q matrix.

  // compute the time elapsed between the current and previous measurements
  auto dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  auto dt_2 = dt * dt;
  auto dt_3 = dt_2 * dt;
  auto dt_4 = dt_3 * dt;

  // 1. Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;

  // 2. Set the process covariance matrix Q
  auto noise_ax = 9;
  auto noise_ay = 9;

  ekf_.Q_ <<  dt_4 / 4 * noise_ax,    0, dt_3 / 2 * noise_ax, 0,
              0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
              dt_3 / 2 * noise_ax, 0, dt_2*noise_ax, 0,
              0, dt_3 / 2 * noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();

  //////////// Update //////////////////////////////////////

  // Use the sensor type to perform the update step.
  // Update the state and covariance matrices.

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    auto ro = measurement_pack.raw_measurements_[0];
    auto theta = measurement_pack.raw_measurements_[1];
    auto rodot = measurement_pack.raw_measurements_[2];

    VectorXd z(3);
    z << ro, theta, rodot;

    Tools tools;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(z);
  }
  else {
    // Laser updates
    auto px = measurement_pack.raw_measurements_[0];
    auto py = measurement_pack.raw_measurements_[1];

    VectorXd z(2);
    z << px, py;

    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(z);
  } 

  // print the output
  //std::cout << "x_ = " << ekf_.x_ << endl;
  //std::cout << "P_ = " << ekf_.P_ << endl;
}
