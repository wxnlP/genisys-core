#pragma once

#include <vector>

class CubicSegment
{
public:
  	CubicSegment(double a0, double a1, double a2, double a3, double t0, double t1) 
        : a0_(a0), a1_(a1), a2_(a2), a3_(a3), t0_(t0), t1_(t1) {}

    bool InTimeSegment(double t) const { return (t >= t0_ && t <= t1_); }
    
    double GetPosition(double t) const {
        double T = t - t0_;
        return (a0_ + a1_ * T + a2_ * (T * T) + a3_ * (T * T * T));
    }

    double GetVelocity(double t) const {
        double T = t - t0_;
        return (a1_ + 2 * a2_ * T + 3 * a3_ * (T * T));
    }

    double GetAcceleration(double t) const {
        double T = t - t0_;
        return (2 * a2_ + 6 * a3_ * T);
    }

private:
    double a0_, a1_, a2_, a3_;
    double t0_, t1_;
};

class CubicTrajectoryPlanner
{
public:
    struct CubicSegPoint_t {
        double pos;
        double vel;
        double acc;
    };

    CubicTrajectoryPlanner() = default;

    bool Plan(const std::vector<double>& theta, const std::vector<double>& time);
    bool Sample(double t, CubicSegPoint_t& point);
private:
    void SolveTridiagonal(const std::vector<double>& a, 
                          const std::vector<double>& b, 
                          const std::vector<double>& c, 
                          const std::vector<double>& d, std::vector<double>& x);
    bool CalculateViaPointVel();
    bool GenerateSegment();
private:
    std::vector<double> theta_;
    std::vector<double> time_;
    std::vector<double> vel_;
    std::vector<CubicSegment> segment_;
};