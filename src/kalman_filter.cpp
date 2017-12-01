#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

const double pi = 3.14159265358979323846;

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft;
  Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // Update for lidar measurements
  VectorXd y = z - H_ * x_;
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // combined estimate
  x_ = x_ + K * y;
  MatrixXd Identity = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (Identity - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */




  // mapping from state space to measurement space
  double rho = sqrt(pow(x_(0), 2) + pow(x_(1), 2));
  // use atan2 to avoid dealing with range issues
  double phi = atan2(x_(1), x_(0));
  double rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;

  // clip predictions too close to zero
  if (fabs(x_(0)) < 1e-4 or fabs(x_(1)) < 1e-4) {
      if (fabs(x_(0)) < 1e-4) {
          x_(0) = 1e-4;
      }

      if (fabs(x_(1)) < 1e-4) {
          x_(1) = 1e-4;
      }
      rho = sqrt(pow(x_(0), 2) + pow(x_(1), 2));
      phi = 0;
      rho_dot = 0;
  }

  VectorXd x_ms_space(3);
  x_ms_space << rho, phi, rho_dot;

  // update step
  VectorXd y = z - x_ms_space;
  // normalize phi
  while (y(1) > pi | y(1) < -pi) {
      if (y(1) > pi) {
          y(1) -= 2 * pi;
      } else if (y(1) < -pi) {
          y(1) += 2 * pi;
      }
  }

  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // combined estimate
  x_ = x_ + K * y;
  MatrixXd Identity = MatrixXd::Identity(x_.size(), x_.size());
  P_ = (Identity - K * H_) * P_;

}
