#include "cubic_time_scaler.h"
#include <cmath>

bool CubicTimeScaler::Plan(const PlanConfig& config)
{
    if (config.T <= 0.0) { return false; }
    duration_ = config.T;
    a0_ = config.s0;
    a1_ = 0.0;
    a2_ = 3.0 * (config.s1 - config.s0) / std::pow(config.T, 2);
    a3_ = -2.0 * (config.s1 - config.s0) / std::pow(config.T, 3);
    return true;
}

bool CubicTimeScaler::Evaluate(double t, State& state) const
{
    if (t < 0.0 || t > duration_) { return false; }
    state.t = t;
    state.s = a0_ + a1_ * t + a2_ * std::pow(t, 2) + a3_ * std::pow(t, 3);
    state.sd = a1_ + 2.0 * a2_ * t + 3.0 * a3_ * std::pow(t, 2);
    state.sdd = 2.0 * a2_ + 6.0 * a3_ * t;  
    return true;
}

double CubicTimeScaler::GetDuration() const
{
    return duration_;
}