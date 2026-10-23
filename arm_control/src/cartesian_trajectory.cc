#include "cartesian_trajectory.h"

#include <Eigen/Geometry>

bool CartesianTrajectory::Sample(
    double t, ArmMotion::CartesianTrajectoryPoint& point) const {
  TimeScalerBase::State state{};
  if (!time_scaler_->Evaluate(t, state)) {
    return false;
  }

  point.t = state.t;

  ArmMotion::Vector3d linear_tangent;
  ArmMotion::Vector3d angular_tangent;
  path_->EvaluatePos(state.s, point.pose);
  path_->EvaluateTangent(linear_tangent, angular_tangent);

  Eigen::Quaterniond q(point.pose.quat.w, point.pose.quat.x,
                       point.pose.quat.y, point.pose.quat.z);
  q.normalize();
  // 角速度切向量由起点系转到基座系
  const Eigen::Vector3d angular_base =
      q.toRotationMatrix() *
      Eigen::Vector3d(angular_tangent.x, angular_tangent.y, angular_tangent.z);

  point.velocity.linear.x = linear_tangent.x * state.sd;
  point.velocity.linear.y = linear_tangent.y * state.sd;
  point.velocity.linear.z = linear_tangent.z * state.sd;
  point.velocity.angular.x = angular_base.x() * state.sd;
  point.velocity.angular.y = angular_base.y() * state.sd;
  point.velocity.angular.z = angular_base.z() * state.sd;

  point.acceleration.linear.x = linear_tangent.x * state.sdd;
  point.acceleration.linear.y = linear_tangent.y * state.sdd;
  point.acceleration.linear.z = linear_tangent.z * state.sdd;
  point.acceleration.angular.x = angular_base.x() * state.sdd;
  point.acceleration.angular.y = angular_base.y() * state.sdd;
  point.acceleration.angular.z = angular_base.z() * state.sdd;

  return true;
}

double CartesianTrajectory::GetDuration() const {
  return time_scaler_->GetDuration();
}
