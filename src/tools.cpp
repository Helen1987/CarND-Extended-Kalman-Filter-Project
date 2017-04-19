#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0, 0, 0, 0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if ((estimations.size() != ground_truth.size()) || (estimations.size() == 0)) {
    std::cout << "The estimation vector size should not be zero and equal ground truth vector size" << std::endl;
    return rmse;
  }

  //accumulate squared residuals
  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd diff = estimations[i] - ground_truth[i];
    diff = diff.array()*diff.array();
    rmse += diff;
  }

  //calculate the mean
  rmse = rmse.array() / estimations.size();
  //calculate the squared root
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

  MatrixXd Hj(3, 4);

  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  double sq_sum = px*px + py*py;
  if (abs(sq_sum) < 0.01) {
    std::cout << "CalculateJacobian () - Errordivision by zero" << std::endl;
    Hj << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
    std::cout << Hj << std::endl;
    return Hj;
  }

  //compute the Jacobian matrix
  Hj(0, 0) = px / sqrt(sq_sum);
  Hj(0, 1) = py / sqrt(sq_sum);
  Hj(0, 2) = 0;
  Hj(0, 3) = 0;
  Hj(1, 0) = -py / sq_sum;
  Hj(1, 1) = px / sq_sum;
  Hj(1, 2) = 0;
  Hj(1, 3) = 0;
  Hj(2, 0) = py*(vx*py - vy*px) / pow(sq_sum, 3. / 2);
  Hj(2, 1) = px*(vy*px - vx*py) / pow(sq_sum, 3. / 2);
  Hj(2, 2) = Hj(0, 0);
  Hj(2, 3) = Hj(0, 1);

  return Hj;

}
