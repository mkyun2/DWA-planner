#include <utility>

class MotionController
{
public:
    MotionController(double wheel_radius, double wheel_separation);

    std::pair<double, double> computeControlInput(double v_d, double w_d, double wl_measure, double wr_measure);
    
private:
    double wheel_radius_;
    double wheel_seperation_;
    std::pair<double, double> computeWheelSpeed(double v_d, double w_d);
};