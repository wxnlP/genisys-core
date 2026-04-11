#pragma once

#include "arm_type.h"
#include "trajectory_linear_planner.h"
#include "trajectory_quaternion_planner.h"

class CartesianTrajectoryPlanner
{
public:
    using TrajectoryPoint_t = TrajectoryPlannerBase::TrajectoryPoint_t;

    struct CartesianPoint_t{
        TrajectoryPoint_t x;
        TrajectoryPoint_t y;
        TrajectoryPoint_t z;
        ArmMotion::Quaternion_t q;
        ArmMotion::Vector3d omega;
    };

    CartesianTrajectoryPlanner(double linear_tb) 
        : tb_(linear_tb), linear_planner_{
            LinearTrajectoryPlanner(tb_),
            LinearTrajectoryPlanner(tb_),
            LinearTrajectoryPlanner(tb_)
        } {}

    bool Plan(const std::vector<ArmMotion::Pose6Dof_t>& path, 
              const std::vector<double>& time);
    bool Sample(double t, CartesianPoint_t& point);
private:
    double tb_;
    LinearTrajectoryPlanner linear_planner_[3];
    QuaternionTrajectoryPlanner quat_planner_;
    std::vector<ArmMotion::Pose6Dof_t> path_;
    std::vector<double> time_;
};
