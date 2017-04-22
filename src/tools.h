#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

class jacobian_calculus : public std::invalid_argument
{
  public:

    jacobian_calculus(const std::string& what_arg) : std::invalid_argument(what_arg) {}
    jacobian_calculus(const char* what_arg) : std::invalid_argument(what_arg) {}
};

class Tools {
  public:

    Tools();
    virtual ~Tools();

    /**
    * A helper method to calculate RMSE.
    */
    Eigen::VectorXd CalculateRMSE(const std::vector<Eigen::VectorXd> &estimations, const std::vector<Eigen::VectorXd> &ground_truth);

    /**
    * A helper method to calculate Jacobians.
    */
    Eigen::MatrixXd CalculateJacobian(const Eigen::VectorXd& x_state);

};

#endif /* TOOLS_H_ */
