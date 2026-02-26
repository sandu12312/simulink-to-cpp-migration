#pragma once

/*
 * KalmanFilter.hpp
 * Modern C++17 rewrite of legacy kalman_filter.c
 * 1D Kalman Filter — same math, better structure
 */

class KalmanFilter
{
public:
    KalmanFilter(double initial_x, double initial_P, double Q, double R)
        : x_(initial_x), P_(initial_P), Q_(Q), R_(R)
    {}

    double update(double measurement)
    {
        /* Predict */
        P_ = P_ + Q_;

        /* Update */
        double K = P_ / (P_ + R_);   /* Kalman gain */
        x_ = x_ + K * (measurement - x_);
        P_ = (1.0 - K) * P_;

        return x_;
    }

    double getEstimate() const { return x_; }
    double getCovariance() const { return P_; }

private:
    double x_;  /* state estimate */
    double P_;  /* error covariance */
    double Q_;  /* process noise */
    double R_;  /* measurement noise */
};
