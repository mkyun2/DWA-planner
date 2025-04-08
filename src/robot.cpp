#include "robot.hpp"
#include <cmath>

Robot::Robot(double x, double y, double yaw,
    double wheel_radius, double wheel_separation)
    :x_(x), y_(y), yaw_(yaw), wheel_radius_(wheel_radius),
     wheel_seperation_(wheel_separation), motor_cmd_left_(0.0), motor_cmd_right_(0.0)
{   
}

double Robot::getPosX()
{
    return x_;
}
double Robot::getPosY()
{
    return y_;
}
double Robot::getYaw()
{
    return yaw_;
}
double Robot::getVelX()
{
    return vel_x_;
}

double Robot::getYawRate()
{
    return yaw_rate_;
}
double Robot::getWheelSpeedLeft()
{
    return wheel_vel_left_;
}
double Robot::getWheelSpeedRight()
{
    return wheel_vel_right_;
}
void Robot::applyMotorCommands(double motor_cmd_left, double motor_cmd_right)
{
    motor_cmd_left_ = motor_cmd_left;
    motor_cmd_right_ = motor_cmd_right;
}

void Robot::update(double dt)
{
    wheel_vel_left_ += motor_cmd_left_ * dt;
    wheel_vel_right_ += motor_cmd_right_ * dt;

    vel_x_ = (wheel_radius_ * 0.5) * (wheel_vel_left_ + wheel_vel_right_);
    yaw_rate_ = (wheel_radius_/wheel_seperation_) * (wheel_vel_right_ - wheel_vel_left_);

    x_ += vel_x_ * std::cos(yaw_) * dt;
    y_ += vel_x_ * std::sin(yaw_) * dt;
    yaw_ += yaw_rate_ * dt;

    if(yaw_ > M_PI) 
        yaw_ -= 2.0*M_PI;
    else if(yaw_ < -M_PI)
        yaw_ += 2.0*M_PI;
}