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
  rmse << 0,0,0,0;

  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  for(int i=0; i < estimations.size(); ++i){
    VectorXd c = estimations[i] - ground_truth[i];
    c = c.array() * c.array();
    rmse = rmse + c;
  }

  rmse /= estimations.size();
  rmse = rmse.array().sqrt();
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  if(px*px+py*py<0.000001){
    std::cout << "Error" << std::endl;
    return Hj;
  }else{
    double s = px*px+py*py;
    double sqrt_s = sqrt(s);
    double s32 = s*sqrt_s;
    Hj << px/sqrt_s, py/sqrt_s, 0, 0,
          -py/s, px/s, 0, 0,
          py*(vx*py-vy*px)/s32, px*(vy*px-vx*py)/s32, px/sqrt_s, py/sqrt_s;
  }
	
  return Hj;
}
