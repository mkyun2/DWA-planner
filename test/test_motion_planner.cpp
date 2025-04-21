#include "motion_planner.hpp"
#include <gtest/gtest.h>

TEST(MotionTest, BasicAssertions){
    //Assertion Example
    //EXPECT_STRNE("hello", "world");
    //EXPECT_EQ(7*6, 42);

    position start{0, 0, 0};
    position goal{1, 0, 0};
    velocity cur_vel{0, 0, 0};
    trajectories trajs;
    /*dt, simulation_time, vel_sample_num, lin_v_max, lin_v_min, acc_max, ang_v_max, ang_acc_max */
    TrajectoryGenerator gen(0.1, 1.0, 30, 1.0, 0.0, 999.0, 1.0, 999.0);
    trajs = gen.generateTrajectories(start, cur_vel);
    EXPECT_EQ(trajs.size(), 900);
    
    gen = TrajectoryGenerator(0.1, 1.0, 30, 1.0, 0.0, 0.0, 1.0, 0.0);
    trajs = gen.generateTrajectories(start, cur_vel);
    EXPECT_EQ(trajs.size(),0);

    gen = TrajectoryGenerator(0.1, 1.0, 1000, 1.0, -1.0, 999.0, 1.0, 999.0);
    trajs = gen.generateTrajectories(start, cur_vel);
    EXPECT_EQ(trajs.size(), 1000000);
    
    gen.setNumOfVelocitySample(0);
    trajs = gen.generateTrajectories(start, cur_vel);
    EXPECT_EQ(trajs.size(), 0);
    
}