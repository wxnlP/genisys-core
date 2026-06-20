#pragma once

#include <memory>

#include "joint_path_base.h"
#include "time_scaler_base.h"

class JointTrajectory {
 public:
  JointTrajectory(std::shared_ptr<JointPathBase> path,
                  std::shared_ptr<TimeScalerBase> time_scaler)
      : path_(std::move(path)), time_scaler_(std::move(time_scaler)) {}

  bool Sample(double t, ArmMotion::JointTrajectoryPoint& point) const;
  double GetDuration() const;

 private:
  std::shared_ptr<JointPathBase> path_;
  std::shared_ptr<TimeScalerBase> time_scaler_;
};