#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
    is_initialized_ = false;

    previous_timestamp_ = 0;

    // initializing matrices
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);

    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    //measurement covariance matrix - radar
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    // set the acceleration noise components
    noise_ax = 9;
    noise_ay = 9;

    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
    /**
     * Initialization
     */
    if (!is_initialized_) {
        IntializeEKF(measurement_pack);

        previous_timestamp_ = measurement_pack.timestamp_;
        is_initialized_ = true;
        return;
    }

    // compute the time elapsed between the current and previous measurements
    // dt - expressed in seconds
    double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
    previous_timestamp_ = measurement_pack.timestamp_;


    // Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    // set the process covariance matrix Q
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;

    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ << dt_4 / 4 * noise_ax, 0, dt_3 / 2 * noise_ax, 0,
            0, dt_4 / 4 * noise_ay, 0, dt_3 / 2 * noise_ay,
            dt_3 / 2 * noise_ax, 0, dt_2 * noise_ax, 0,
            0, dt_3 / 2 * noise_ay, 0, dt_2 * noise_ay;

    /**
     * Prediction
     */

    ekf_.Predict();

    /**
     * Update
     */

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
          ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);

    } else {
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}

void FusionEKF::IntializeEKF(const MeasurementPackage &measurement_pack) {
    ekf_ = KalmanFilter();

    //transition matrix
    MatrixXd F = MatrixXd(4, 4);
    F << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

    //state covariance matrix
    MatrixXd P = MatrixXd(4, 4);
    P << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    MatrixXd Q = MatrixXd(4, 4);

    // first measurement
    cout << "EKF: " << endl;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

        VectorXd x_in = tools.ConvertPolarToCartesian(measurement_pack.raw_measurements_);
        ekf_.Init(x_in,
                  P,
                  F,
                  H_laser_,
                  R_radar_,
                  Q);

    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
        VectorXd x_in = VectorXd(4);
        x_in << measurement_pack.raw_measurements_[0],
                measurement_pack.raw_measurements_[1],
                0,
                0;

        ekf_.Init(x_in,
                  P,
                  F,
                  H_laser_,
                  R_laser_,
                  Q);

    }
}
