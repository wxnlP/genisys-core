#pragma once

#include "trajectory_planner_base.h"

class LinearTrajectoryPlanner : public TrajectoryPlannerBase
{
public:

    LinearTrajectoryPlanner(double tb) : tb_(tb) {}

    bool Plan(const std::vector<double>& theta, 
              const std::vector<double>& time) override;
private:
    double GetPosition(double t) const override;
    double GetVelocity(double t) const override;
    double GetAcceleration(double t) const override;
private:
    std::vector<double> vel_;
    std::vector<double> accel_;
    std::vector<double> pos_pre_blend_;
    std::vector<double> pos_post_blend_;
    double start_vel_{0.0};  // 起始速度
    double end_vel_{0.0};    // 终止速度
    double tb_;
};