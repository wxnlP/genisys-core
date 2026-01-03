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

// class SegmentBase
// {
// public:
//     struct TrajectoryPoint_t {
//         double pos;
//         double vel;
//         double acc;
//     };

//     SegmentBase(double t0, double t1) : t0_(t0), t1_(t1) {}
//     virtual ~SegmentBase() = default;

//     virtual bool InTimeSegment(double t) const = 0;
//     virtual double GetPosition(double t) const = 0;
//     virtual double GetVelocity(double t) const = 0;
//     virtual double GetAcceleration(double t) const = 0;

//     double t0_, t1_;
// };

// class CubicSegment : public SegmentBase
// {
// public:
//   	CubicSegment(double a0, double a1, double a2, double a3, double t0, double t1) 
//         : SegmentBase(t0, t1), a0_(a0), a1_(a1), a2_(a2), a3_(a3) {}

//     bool InTimeSegment(double t) const override { return (t >= t0_ && t <= t1_); }
    
//     double GetPosition(double t) const override {
//         double T = t - t0_;
//         return (a0_ + a1_ * T + a2_ * (T * T) + a3_ * (T * T * T));
//     }

//     double GetVelocity(double t) const override {
//         double T = t - t0_;
//         return (a1_ + 2 * a2_ * T + 3 * a3_ * (T * T));
//     }

//     double GetAcceleration(double t) const override {
//         double T = t - t0_;
//         return (2 * a2_ + 6 * a3_ * T);
//     }

// private:
//     double a0_, a1_, a2_, a3_;
// };

