#pragma once

#include <memory>

#include "cartesian_path_base.h"
#include "time_scaler_base.h"

class CartesianTrajectory {
 public:
  CartesianTrajectory(std::shared_ptr<CartesianPathBase> path,
                      std::shared_ptr<TimeScalerBase> time_scaler)
      : path_(std::move(path)), time_scaler_(std::move(time_scaler)) {}

  bool Sample(double t, ArmMotion::CartesianTrajectoryPoint& point) const;
  double GetDuration() const;

 private:
  std::shared_ptr<CartesianPathBase> path_;
  std::shared_ptr<TimeScalerBase> time_scaler_;
};