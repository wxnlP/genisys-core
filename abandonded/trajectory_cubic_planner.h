#pragma once

#include "trajectory_planner_base.h"


class CubicTrajectoryPlanner : public TrajectoryPlannerBase
{
public:

    CubicTrajectoryPlanner() = default;

    bool Plan(const std::vector<double>& theta, 
              const std::vector<double>& time) override;
private:
    double GetPosition(double t) const override;
    double GetVelocity(double t) const override;
    double GetAcceleration(double t) const override;
    void SolveTridiagonal(const std::vector<double>& a, 
                          const std::vector<double>& b, 
                          const std::vector<double>& c, 
                          const std::vector<double>& d, std::vector<double>& x);
    bool CalculateViaPointVel();
    bool GenerateSegment();
private:
    struct CubicSegment_t {
        double a0;
        double a1;
        double a2;
        double a3;
    };
    std::vector<double> vel_;
    std::vector<CubicSegment_t> segment_;
};

