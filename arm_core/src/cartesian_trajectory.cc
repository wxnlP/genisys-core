#include "cartesian_trajectory.h"

bool CartesianTrajectory::Sample(
    double t, ArmMotion::CartesianTrajectoryPoint& point) const {
  TimeScalerBase::State state{};
  if (!time_scaler_.Evaluate(t, state)) {
    return false;
  }

  point.t = state.t;

  ArmMotion::Vector3d linear_tangent;
  ArmMotion::Vector3d angular_tangent;
  path_.EvaluatePos(state.s, point.pose);
  path_.EvaluateTangent(linear_tangent, angular_tangent);

  point.velocity.linear.x = linear_tangent.x * state.sd;
  point.velocity.linear.y = linear_tangent.y * state.sd;
  point.velocity.linear.z = linear_tangent.z * state.sd;
  point.velocity.angular.x = angular_tangent.x * state.sd;
  point.velocity.angular.y = angular_tangent.y * state.sd;
  point.velocity.angular.z = angular_tangent.z * state.sd;

  point.acceleration.linear.x = linear_tangent.x * state.sdd;
  point.acceleration.linear.y = linear_tangent.y * state.sdd;
  point.acceleration.linear.z = linear_tangent.z * state.sdd;
  point.acceleration.angular.x = angular_tangent.x * state.sdd;
  point.acceleration.angular.y = angular_tangent.y * state.sdd;
  point.acceleration.angular.z = angular_tangent.z * state.sdd;

  return true;
}

double CartesianTrajectory::GetDuration() const {
  return time_scaler_.GetDuration();
}
