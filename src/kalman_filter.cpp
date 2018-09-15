#include "kalman_filter.h"
#include <math.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(MatrixXd &H_in, MatrixXd &RL_in, MatrixXd &RR_in) {
  H_ = H_in;
  RL_ = RL_in;
  RR_ = RR_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  P_ = F_* P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + RL_;
  MatrixXd K = P_ * Ht * S.inverse();
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  x_ = x_ + (K * y);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  double rho, phi, rhodot;
  double px, py, vx, vy;

  // Calculate y
  px = x_(0);
  py = x_(1);
  vx = x_(2);
  vy = x_(3);
  rho = sqrt(px*px+py*py);
  if(rho<0.001){
    cout << "Error: in EKF, divided by zero!" << endl;
  }
  phi = atan2(py, px);
  rhodot = (px*vx+py*vy)/rho;
  double y0 = z(0) - rho;
  double y1 = z(1) - phi;
  double y2 = z(2) - rhodot;
  if(y1>3.1415926){
    y1 = y1 - 2 * 3.1415926;
  }
  if(y1<-3.1415926){
    y1 = y1 + 2 * 3.1415926;
  }
  VectorXd y(3);
  y << y0, y1, y2;

  // Calculate H
  double rho2 = rho * rho;
  double rho32 = rho2 * rho;

  // Update
  MatrixXd Ht = Hj_.transpose();
  MatrixXd S = Hj_ * P_ * Ht + RR_;
  MatrixXd K = P_ * Ht * S.inverse();
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  x_ = x_ + (K * y);
  P_ = (I - K * Hj_) * P_;

}
