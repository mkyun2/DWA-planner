#include "robot.hpp"
#include "motion_controller.hpp"
#include "motion_planner.hpp"
#include <iostream>
#include <vector>

#include <fstream>

int main()
{
    double traj_gen_dt = 0.1;
    double traj_gen_time = 1.0;
    double wheel_radius = 0.05;
    double wheel_separation = 0.20;
    double linear_vel_max = 1.0;
    double linear_vel_min = 0.0;
    double angular_vel_max = 0.6;
    double linear_acc_max = 2.0;
    double angular_acc_max = 2.5;
    Robot robot(0.0, 0.0, 0.0, wheel_radius, wheel_separation);
    TrajectoryGenerator traj_gen(traj_gen_dt, traj_gen_time, linear_vel_max, linear_vel_min, linear_acc_max, angular_vel_max, angular_acc_max);
    MotionController controller(wheel_radius, wheel_separation);
    double dt = 0.01;
    double sim_time = 5.0;
    int steps = static_cast<int>(sim_time/dt);
    position goal;
    goal.x = 1.0;
    goal.y = 1.0;

    std::vector<position> obstacles;
    obstacles.push_back({0.3, 0.3, 0.0});
    obstacles.push_back({0.5, 0.5, 0.0});
    obstacles.push_back({0.7, 0.5, 0.0});
    obstacles.push_back({0.5, 0.8, 0.0});
    std::ofstream fout("environment.csv");
    fout << "goal_x,goal_y,obs_x,obs_y\n";
    for(auto obs:obstacles)
    {
        fout << goal.x << "," << goal.y << "," << obs.x << "," << obs.y << "\n";
    }
    fout.close();
    
    //std::ofstream fout("robot_trajectory.csv");
    fout.open("robot_trajectory.csv");
    fout << "time_step,step_idx,x,y,cur_x,cur_y\n";
    for(int i = 0; i<steps; ++i)
    {
        std::vector<std::pair<std::vector<position>, velocity>> trajectories;
        position cur_pos;
        velocity cur_vel;
        double t = i*dt;
        
        
        cur_pos.x = robot.getPosX();
        cur_pos.y = robot.getPosY();
        cur_pos.yaw = robot.getYaw();

        cur_vel.x = robot.getVelX();
        cur_vel.yaw = robot.getYawRate();
        
        trajectories = traj_gen.generateTrajectories(cur_pos, cur_vel);
        TrajectoryEvaluator traj_eval(cur_pos, cur_vel, goal);
        auto [traj_, vel_] = traj_eval.evaluateTrajectories(trajectories, obstacles);
        //auto [v_d, w_d] = planner.getVelocityCommand();
        double wl_measure = robot.getWheelSpeedLeft();
        double wr_measure = robot.getWheelSpeedRight();
        auto [motor_left_cmd, motor_right_cmd] = controller.computeControlInput(vel_.x,vel_.yaw, wl_measure, wr_measure);
        //auto [motor_left_cmd, motor_right_cmd] = controller.computeControlInput(v_d,w_d, wl_measure, wr_measure);
        robot.applyMotorCommands(motor_left_cmd, motor_right_cmd);
        robot.update(dt);
        
        std::cout << i <<"vel ["<< robot.getVelX() << ", " << robot.getYawRate() << "]" << std::endl;
        std::cout << i <<"Pos ["<< robot.getPosX() << ", " << robot.getPosY() << ", " << robot.getYaw() << "]"<< std::endl;
        
        //Print Simulation Data
        
        for(int n_pt = 0; n_pt < traj_.size(); n_pt++)
        {
            const auto& p = traj_[n_pt];
            fout << i << "," << n_pt << ","
            << p.x << "," << p.y << "," << cur_pos.x << "," << cur_pos.y << "\n";
        }
        

    }
    fout.close();
}