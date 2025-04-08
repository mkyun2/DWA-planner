#include "motion_controller.hpp"
MotionController::MotionController(double wheel_radius, double wheel_separation)
: wheel_radius_(wheel_radius), wheel_seperation_(wheel_separation)
{    
}
std::pair<double, double> MotionController::computeWheelSpeed(double v_d, double w_d)
{
    double wr_d = v_d / wheel_radius_+ wheel_seperation_ / (2.0 * wheel_radius_) * w_d;
    double wl_d = v_d / wheel_radius_- wheel_seperation_ / (2.0 * wheel_radius_) * w_d;
    return std::make_pair(wl_d, wr_d);
}
std::pair<double, double> MotionController::computeControlInput(double v_d, double w_d, double wl_measure, double wr_measure)
{
    double kp = 1.0;
    // double ki = 1.0;
    // double kd = 1.0;
    auto [wl_d, wr_d] = computeWheelSpeed(v_d,w_d);
    double error_w_left = wl_d - wl_measure;
    double error_w_right = wr_d - wr_measure;
    
    double control_w_left = kp*error_w_left ;
    double control_w_right = kp*error_w_right;

    return std::make_pair(control_w_left,control_w_right);
}