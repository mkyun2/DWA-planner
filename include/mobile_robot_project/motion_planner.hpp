#include <utility>
#include <vector>
#include "utils.hpp"
// class MotionPlanner
// {
// public:
//     std::pair<double, double> getVelocityCommand();
// };

class TrajectoryGenerator
{
public:
    TrajectoryGenerator(double dt, double simulation_time, double linear_vel_max, double linear_vel_min, double linear_acc_max, double angular_vel_max, double angular_acc_max); 
    std::vector<std::pair<std::vector<position>,velocity>> generateTrajectories(position current_pos, velocity current_vel);
private:
    double dt_, simulation_time_;
    double linear_vel_max_, linear_vel_min_;
    double linear_acc_max_;

    double angular_vel_max_;
    double angular_acc_max_;
    std::vector<velocity> sampleVelocity();
    void removeUnrechableVelocity(std::vector<velocity> & velocities, velocity current_vel);

};

class TrajectoryEvaluator
{
public:
    TrajectoryEvaluator(const position pos, const velocity vel, const position goal);
    
    std::pair<std::vector<position>,velocity> evaluateTrajectories(const std::vector<std::pair<std::vector<position>,velocity>> & trajecories, const std::vector<position> & obstacles);
private:
    position pose_;
    velocity vel_;
    position goal_;

    double max_speed_;
    double penalty_goal_;
    double penalty_speed_;
    double penalty_obstacle_;

    double evaluateGoal(const std::vector<position> & traj);
    double evaluateSpeed(const std::vector<position> & traj, const velocity & vel);
    double evaluateObstacle(const std::vector<position> & traj, const std::vector<position> & obstacles);

};