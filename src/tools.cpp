#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        std::cout << "Invalid estimation or ground_truth data" << std::endl;
        return rmse;
    }

    // accumulate squared residuals0
    for (unsigned int i=0; i < estimations.size(); ++i) {
        VectorXd residual = estimations[i] - ground_truth[i];

        // coefficient-wise multiplication
        residual = residual.array()*residual.array();
        rmse += residual;
    }

    // calculate the mean
    rmse = rmse/estimations.size();

    // calculate the squared root
    rmse = rmse.array().sqrt();

    // return the result
    std::cout << "RMSE " << rmse << std::endl;
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);
    // recover state parameters
    double px = x_state(0);
    double py = x_state(1);
    double vx = x_state(2);
    double vy = x_state(3);

    // pre-compute a set of terms to avoid repeated calculation
    double c1 = px*px+py*py;
    double c2 = sqrt(c1);
    double c3 = (c1*c2);

    // check division by zero
    if (fabs(c1) < 0.0001) {
        std::cout << "CalculateJacobian () - Error - Division by Zero" << std::endl;
        return Hj;
    }

    // compute the Jacobian matrix
    Hj <<   (px/c2), (py/c2), 0, 0,
            -(py/c1), (px/c1), 0, 0,
            py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3,
            px/c2, py/c2;

    return Hj;
}

double Tools::NormalizeAngle(double angle){
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

Eigen::VectorXd Tools::ConvertPolarToCartesian(const Eigen::VectorXd& v){

    VectorXd cartesian_vec = VectorXd (4);

    const double rho = v(0);
    const double phi = v(1);

    const double px = rho * cos(phi);
    const double py = rho * sin(phi);

    cartesian_vec << px, py, 0, 0;

    return cartesian_vec;
}

Eigen::VectorXd Tools::ConvertCartesianToPolar(const Eigen::VectorXd& v){
    const double px = v[0];
    const double py = v[1];

    const double vx = v[2];
    const double vy = v[3];

    const double px_pow_square = sqrt(px * px + py * py);
    VectorXd polar_vec = VectorXd(3);
    polar_vec << px_pow_square,
            atan2(py, px),
            (px * vx + py * vy) / px_pow_square;

    std::cout << "cartesian to polar " << polar_vec << std::endl;
    return polar_vec;

}

