#include "joint_trajectory.h"

bool JointTrajectory::Sample(double t,
                             ArmMotion::JointTrajectoryPoint_t& point) const {
  TimeScalerBase::State state{};
  if (!time_scaler_.Evaluate(t, state)) {
    return false;
  }

  point.t = state.t;
  point.s = state.s;
  point.sd = state.sd;
  point.sdd = state.sdd;

  ArmMotion::Joint6Dof_t tangent;
  path_.EvaluatePos(state.s, point.position);
  path_.EvaluateTangent(tangent);

  for (int i = 0; i < 6; ++i) {
    point.velocity.q[i] = tangent.q[i] * state.sd;
    point.acceleration.q[i] = tangent.q[i] * state.sdd;
  }

  return true;
}

double JointTrajectory::GetDuration() const {
  return time_scaler_.GetDuration();
}
