#pragma once

#include <vector>

class TrajectoryPlannerBase
{
public:
    struct TrajectoryPoint_t {
        double pos;
        double vel;
        double acc;
    };

    TrajectoryPlannerBase() = default;
    virtual ~TrajectoryPlannerBase() = default;

    virtual bool Plan(const std::vector<double>& theta, 
                      const std::vector<double>& time) = 0;
    virtual bool Sample(double t, TrajectoryPoint_t& point) 
    {
        if (t <= time_.front()) {
            point.pos = theta_.front();
            point.acc = 0.0;
            point.vel = 0.0;
            return true;
        }
        if (t >= time_.back()) {
            point.pos = theta_.back();
            point.acc = 0.0;
            point.vel = 0.0;
            return true;
        }

        point.pos = GetPosition(t);
        point.vel = GetVelocity(t);
        point.acc = GetAcceleration(t);
        
        return true;
    }
    virtual double GetPosition(double t) const = 0;
    virtual double GetVelocity(double t) const = 0;
    virtual double GetAcceleration(double t) const = 0;
protected:
    std::vector<double> theta_;
    std::vector<double> time_;
};
