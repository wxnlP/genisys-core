#pragma once

#include "trajectory_planner_base.h"

class LinearTrajectoryPlanner : public TrajectoryPlannerBase
{
public:

    LinearTrajectoryPlanner() = default;

    bool Plan(const std::vector<double>& theta, 
              const std::vector<double>& time) override;
    void SetAccel(double a);
private:
    double GetPosition(double t) const override;
    double GetVelocity(double t) const override;
    double GetAcceleration(double t) const override;
private:
    std::vector<double> vel_;
    std::vector<double> tb_;
    double accel_;
};