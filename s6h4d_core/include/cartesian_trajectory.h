#pragma once

#include "cartesian_path_base.h"
#include "time_scaler_base.h"

class CartesianTrajectory {
 public:
  CartesianTrajectory(const CartesianPathBase& path,
                      const TimeScalerBase& time_scaler)
      : path_(path), time_scaler_(time_scaler) {}

  bool Sample(double t, ArmMotion::CartesianTrajectoryPoint_t& point) const;
  double GetDuration() const;

 private:
  const CartesianPathBase& path_;
  const TimeScalerBase& time_scaler_;
};