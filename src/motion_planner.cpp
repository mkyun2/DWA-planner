#include <random>
#include <limits>
#include <iostream>
#include "motion_planner.hpp"

TrajectoryGenerator::TrajectoryGenerator(double dt, double simulation_time, double linear_vel_max, double linear_vel_min, double linear_acc_max, double angular_vel_max, double angular_acc_max)
:dt_(dt), simulation_time_(simulation_time), linear_vel_max_(linear_vel_max), linear_vel_min_(linear_vel_min), linear_acc_max_(linear_acc_max), angular_vel_max_(angular_vel_max), angular_acc_max_(angular_acc_max)
{    
}
// std::pair<double, double> MotionPlanner::getVelocityCommand()
// {
//     double v_d = 0.2;
//     double w_d = 0.4;
    

//     return std::make_pair(v_d,w_d);
// }

std::vector<std::pair<std::vector<position>,velocity>> TrajectoryGenerator::generateTrajectories(position current_pos, velocity current_vel)
{
    std::vector<velocity> velocities;
    velocities = sampleVelocity();
    removeUnrechableVelocity(velocities, current_vel);
    int steps = simulation_time_ / dt_;
    
    
    std::vector<std::pair<std::vector<position>,velocity>> trajectories;
    for(auto vel : velocities)
    {
        std::vector<position> trajectory;
        position simulated_pos = current_pos;
        trajectory.push_back(current_pos);
        for(int step = 0; step<steps; step++)
        {
            simulated_pos.x +=  vel.x * cosf(simulated_pos.yaw) * dt_;
            simulated_pos.y +=  vel.x * sinf(simulated_pos.yaw) * dt_;
            simulated_pos.yaw +=  vel.yaw * dt_;
            
            trajectory.push_back(simulated_pos);   
        }
        trajectories.push_back(std::make_pair(trajectory, vel));
    }

    return trajectories;
}

std::vector<velocity> TrajectoryGenerator::sampleVelocity()
{
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::uniform_real_distribution<double> linear_vel_sample(linear_vel_min_, linear_vel_max_);
    // std::uniform_real_distribution<double> angular_vel_sample(-angular_vel_max_, angular_vel_max_);
    std::vector<velocity> velocities;
    velocity sampled_velocity = {0.0, 0.0, 0.0};
    int sample_num = 30;
    double linvel_sample_range = (linear_vel_max_ - linear_vel_min_);
    double angvel_sample_range = (2.0 * angular_vel_max_);
    double dv = linvel_sample_range/static_cast<double>(sample_num);
    double dw = angvel_sample_range/static_cast<double>(sample_num);
    for(int i = 0; i < sample_num; i++ )
    {
        sampled_velocity.x += dv;
        sampled_velocity.yaw = -angular_vel_max_;
        for(int j = 0; j < sample_num; j++)
        {
            sampled_velocity.yaw += dw;
            velocities.push_back(sampled_velocity);
        }
    }

    return velocities;
}
void TrajectoryGenerator::removeUnrechableVelocity(std::vector<velocity> & velocities, velocity current_vel)
{
    
    double lin_vel_max = std::min(linear_vel_max_, current_vel.x + linear_acc_max_ * dt_);
    double lin_vel_min = std::max(linear_vel_min_, current_vel.x - linear_acc_max_ * dt_);
    double ang_vel_max = std::min(angular_vel_max_, current_vel.yaw + angular_acc_max_ * dt_);
    double ang_vel_min = std::max(-angular_vel_max_, current_vel.yaw - angular_acc_max_ * dt_);
    std::cout << "sampledVelocity size " << velocities.size() << std::endl;
    std::cout << "currentVelocity:[" << current_vel.x << ", " << current_vel.yaw << "]" << std::endl;
    std::cout << "dynamicWindow linear:["<< lin_vel_min << "~" << lin_vel_max << "], angular[" << ang_vel_min << "~" << ang_vel_max << "]" <<std::endl; 
    for(auto it = velocities.begin(); it != velocities.end();)
    {
        bool erase_sign = false;
        if(it->x > lin_vel_max || it->x < lin_vel_min)
            erase_sign = true;
        if(it->yaw > ang_vel_max || it->yaw < ang_vel_min)
            erase_sign = true;
        if(erase_sign)
            it = velocities.erase(it);
        else
            ++it;
    }
    std::cout << "Velocity size after erasing " << velocities.size() << std::endl;
}

TrajectoryEvaluator::TrajectoryEvaluator(const position pos, const velocity vel, const position goal)
: pose_(pos), vel_(vel), goal_(goal)
{
    penalty_goal_ = 0.5;
    penalty_speed_ = 4.0;
    penalty_obstacle_ = 0.1;
    max_speed_ = 1.0;
}
std::pair<std::vector<position>,velocity> TrajectoryEvaluator::evaluateTrajectories(const std::vector<std::pair<std::vector<position>,velocity>> & trajecories, const std::vector<position> & obstacles)
{
    std::vector<double> traj_cost;
    double best_cost = std::numeric_limits<double>::max();
    int best_index = 0;
    int index = 0;
    
    for(auto traj : trajecories)
    {
        double cost_sum = 0.0;
        cost_sum += penalty_goal_*evaluateGoal(traj.first);
        cost_sum += penalty_speed_*evaluateSpeed(traj.first, traj.second);
        cost_sum += penalty_obstacle_*evaluateObstacle(traj.first, obstacles);
        traj_cost.push_back(cost_sum);
        if(best_cost > cost_sum)
        {
            best_cost = cost_sum;
            best_index = index;
        }
        index++;
    }
    
    return trajecories[best_index];
}

double TrajectoryEvaluator::evaluateGoal(const std::vector<position> & traj)
{
    double error_x = goal_.x - traj.back().x;
    double error_y = goal_.y - traj.back().y;
    double heading = atan2f(error_y,error_x);
    double error_yaw = heading - traj.back().yaw;

    double dist = fabs(atan2f(sinf(error_yaw), cosf(error_yaw)));
    return 1.0 * dist;
}
double TrajectoryEvaluator::evaluateSpeed(const std::vector<position> & traj, const velocity & vel)
{

    double velocity_x = vel.x*cosf(traj.front().yaw);
    double velocity_y = vel.y*sinf(traj.front().yaw);
    double speed = std::hypot(velocity_x,velocity_y);
    
    double dist = std::hypot(goal_.x - traj.back().x, goal_.y - traj.back().y);
    double gain_decel = 1/(1+std::exp(-30*(dist-0.2)));
    double error = fabs(gain_decel * max_speed_ - speed);
    
    return error;

}
double TrajectoryEvaluator::evaluateObstacle(const std::vector<position> & traj, const std::vector<position> & obstacles)
{
    double size = 0.1;
    double min_dist = std::numeric_limits<double>::max();
    double cost = 0.0;   
    
    for(auto obs : obstacles)
    {
        double err_x = obs.x - pose_.x;
        double err_y = obs.y - pose_.y;
        double chk_dist = std::sqrt(err_x * err_x + err_y * err_y);
        //if(chk_dist > 0.5)
        //    continue;
        for(auto pt : traj)
        {
        
            double error_x  = obs.x - pt.x;
            double error_y  = obs.y - pt.y;

            chk_dist = std::sqrt(error_x * error_x + error_y * error_y);
            if(chk_dist < size)
            {
                min_dist = std::numeric_limits<double>::max();
                return min_dist;
            }

            if(chk_dist < min_dist)
                min_dist = chk_dist;
        }
        cost = 1.0/min_dist;
    }
    return cost;
}