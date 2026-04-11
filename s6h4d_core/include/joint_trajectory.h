#pragma once

#include "joint_path_base.h"
#include "time_scaler_base.h"

class JointTrajectory {
public:
    JointTrajectory(const JointPathBase& path, const TimeScalerBase& time_scaler)
        : path_(path), time_scaler_(time_scaler) {}

    bool Sample(double t, ArmMotion::JointTrajectoryPoint_t& point) const;
    double GetDuration() const;

private:
    const JointPathBase& path_;
    const TimeScalerBase& time_scaler_;
};