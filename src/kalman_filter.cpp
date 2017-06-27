#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

const double TWO_PI = 2 * M_PI;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
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
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  // map the predicted state into the measurement space

  auto ro_pred = sqrt(x_[0] * x_[0] + x_[1] * x_[1]);

  auto theta_pred = (fabs(x_[0]) > 0.001) ? atan2(x_[1], x_[0]) : 0.001;
  
  auto rodot_pred = (fabs(ro_pred) > 0.001) ? (x_[0] * x_[2] + x_[1] * x_[3]) / ro_pred : 0.001;

  VectorXd z_pred(3);
  z_pred << ro_pred, theta_pred, rodot_pred;

  VectorXd y = z - z_pred;

  // Angle normalization if necessary:
  if (y[1] < -M_PI || M_PI < y[1])
  {
    // This code was taken from:
    // https://github.com/apache/commons-math/blob/53ec46ba272e23c0c96ada42f26f4e70e96f3115/src/main/java/org/apache/commons/math4/util/MathUtils.java#L107
    y[1] = y[1] - TWO_PI * floor((y[1] + M_PI ) / TWO_PI);
  }

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
