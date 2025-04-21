#include <utility>
#include <vector>
#include "utils.hpp"
// class MotionPlanner
// {
// public:
//     std::pair<double, double> getVelocityCommand();
// };
typedef std::pair<std::vector<position>, velocity> trajectory;
typedef std::vector<std::pair<std::vector<position>, velocity>> trajectories;
typedef std::vector<position> path;
class TrajectoryGenerator
{

public:
    TrajectoryGenerator(double dt, double simulation_time, int vel_sample_num, double linear_vel_max, double linear_vel_min, double linear_acc_max, double angular_vel_max, double angular_acc_max); 
    void setNumOfVelocitySample(int sample_num);
    trajectories generateTrajectories(position current_pos, velocity current_vel);
private:
    double dt_, simulation_time_;
    double linear_vel_max_, linear_vel_min_;
    double linear_acc_max_;

    double angular_vel_max_;
    double angular_acc_max_;
    
    int sample_num_;
    std::vector<velocity> sampleVelocity();
    void removeUnrechableVelocity(std::vector<velocity> & velocities, velocity current_vel);
    
};

class TrajectoryEvaluator
{
public:
    TrajectoryEvaluator(const position pos, const velocity vel, const position goal);
    
    trajectory evaluateTrajectories(const trajectories & trajecories, const std::vector<position> & obstacles);
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