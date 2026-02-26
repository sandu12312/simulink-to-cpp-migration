#pragma once

/*
 * PidController.hpp
 * Modern C++17 rewrite of the legacy pid_controller.c
 *
 * Why this is better than the C version:
 *   - No global state: each object has its own state
 *   - You can create multiple independent PID instances
 *   - Easy to unit test
 *   - Parameters set once in constructor, not passed every step
 */

class PidController
{
public:
    PidController(double kp, double ki, double kd, double dt)
        : kp_(kp), ki_(ki), kd_(kd), dt_(dt),
          prev_error_(0.0), integral_(0.0)
    {}

    double step(double setpoint, double measured)
    {
        double error      = setpoint - measured;
        double derivative = (error - prev_error_) / dt_;

        integral_  += error * dt_;
        prev_error_ = error;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

    void reset()
    {
        prev_error_ = 0.0;
        integral_   = 0.0;
    }

private:
    double kp_, ki_, kd_, dt_;
    double prev_error_;
    double integral_;
};
