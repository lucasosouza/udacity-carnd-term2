#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; //state
  P_ = P_in; //covariance matrix
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
  x_ = F_ * x_ ; // no need u, noise is 0
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */

  // measurement update
  VectorXd z_pred = H_ * x_; // state * measurement
  VectorXd y = z - z_pred; // error
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // new state
  x_ = x_ + (K * y);

  // define identity matrix, square and with same size as state matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  // measurement update
  VectorXd z_pred = tools.CartesianToPolar(x_);
  VectorXd y = z - z_pred; // error
  MatrixXd S = H_ * P_ * H_.transpose() + R_;
  MatrixXd K = P_ * H_.transpose() * S.inverse();

  // new state
  x_ = x_ + (K * y);

  // define identity matrix, square and with same size as state matrix
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * H_) * P_;


}
