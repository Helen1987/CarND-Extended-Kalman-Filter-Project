#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

VectorXd KalmanFilter::ConvertToPolar(double px, double py, double v_x, double v_y) {
  VectorXd f_x = VectorXd(3);
  double ro = sqrt(px*px + py*py);
  f_x << ro, atan2(py, px), (px*v_x + py*v_y) / ro;
  return f_x;
}

void KalmanFilter::UpdateStep(const VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_ * PHt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;

  //new state
  x_ = x_ + (K * y);
  long x_size = x_.size();
  P_ = (MatrixXd::Identity(x_size, x_size) - K * H_) * P_;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  // KF Measurement update step
  VectorXd y = z - H_ * x_;
  UpdateStep(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  auto f_x = ConvertToPolar(x_(0), x_(1), x_(2), x_(3));

  VectorXd y = z - f_x;
  // make sure y_phi is between -pi and pi
  if (y(1) > M_PI) {
    y(1) -= 2 * M_PI;
  }
  else if (y(1) < -M_PI) {
    y(1) += 2 * M_PI;
  }

  UpdateStep(y);
}
