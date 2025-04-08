
class Robot
{
public:
    Robot(double x, double y, double yaw,
          double wheel_radius, double wheel_separation);
    double getPosX();
    double getPosY();
    double getYaw();
    double getVelX();
    double getYawRate();
    double getWheelSpeedLeft();
    double getWheelSpeedRight();
    void applyMotorCommands(double motor_cmd_left, double motor_cmd_right);
    void update(double dt);

private:
    double x_;
    double y_;
    double yaw_;
    
    double vel_x_;
    double vel_y_;
    double yaw_rate_;

    double wheel_vel_left_;
    double wheel_vel_right_;

    double wheel_radius_;
    double wheel_seperation_;

    double motor_cmd_left_;
    double motor_cmd_right_;
};