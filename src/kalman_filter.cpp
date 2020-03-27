#include <iostream>
#include "kalman_filter.h"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
    tools = Tools();
}

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
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
    VectorXd z_pred = H_ * x_;

    std::cout << "new measurement " << z << " z predicted " << x_ << std::endl;
    VectorXd y = z - z_pred;

    DoUpdate(y, H_);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {

    VectorXd h_x_prim = tools.ConvertCartesianToPolar(x_);
    VectorXd y = z - h_x_prim;
    y << y[0], tools.NormalizeAngle(y[1]),  y[2];

    MatrixXd Hj = tools.CalculateJacobian(x_);
    DoUpdate(y, Hj);
}

void KalmanFilter::DoUpdate(const Eigen::VectorXd &y, const Eigen::MatrixXd &H) {
    MatrixXd Ht = H.transpose();
    MatrixXd S = H * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;


    //new estimate
    x_ = x_ + (K * y);

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H) * P_;
}
