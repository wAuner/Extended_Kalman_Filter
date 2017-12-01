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
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  unsigned long n = estimations.size();
  for (int i=0; i < n; i++){
      rmse += (estimations[i] - ground_truth[i]).array().pow(2).matrix();
  }

  return (rmse / n).array().sqrt();

}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    // recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    // check division by zero
    if (fabs(px) < 1e-4) {
        px = 1e-4;
    }

    if (fabs(py) < 1e-4) {
        py = 1e-4;
    }

    double pssum = pow(px, 2) + pow(py, 2);
    if (fabs(pssum) < 1e-3) {
        cout << "Division by zero error \n";
        pssum = 1e-3;
    }
    double psqrt = sqrt(pssum);

    MatrixXd Hj(3,4);
    //compute the Jacobian matrix
    Hj << px / psqrt, py / psqrt, 0, 0,
          -py / pssum, px / pssum, 0, 0,
          py * (vx*py - vy*px) / pow(pssum, 3.0/2), px * (vy*px - vx*py) / pow(pssum, 3.0/2), px / psqrt, py/ psqrt;

    return Hj;

}
