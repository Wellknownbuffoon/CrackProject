#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <ros/ros.h>
class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd)
        : Kp_(Kp), Ki_(Ki), Kd_(Kd),
          prev_error_(0.0), integral_(0.0), initialized_(false) {}

    double compute(double error, ros::Time current_time) {
        // Initialize on first call
        if (!initialized_) {
            prev_time_ = current_time;
            prev_error_ = error;
            initialized_ = true;
            return 0.0;
        }

        double dt = (current_time - prev_time_).toSec();
        prev_time_ = current_time;

        if (dt <= 0.0) return 0.0; // avoid divide by zero

        // PID terms
        double P = Kp_ * error;

        integral_ += error * dt;
        double I = Ki_ * integral_;

        double derivative = (error - prev_error_) / dt;
        double D = Kd_ * derivative;

        prev_error_ = error;

        return P + I + D;
    }

    void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        initialized_ = false;
    }

private:
    double Kp_, Ki_, Kd_;
    double prev_error_, integral_;
    ros::Time prev_time_;
    bool initialized_;
};

#endif