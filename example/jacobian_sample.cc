#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "jacobian.h"

namespace {

std::array<double, 6> ToArray(const ArmMotion::Joint6Dof& joint) {
  return {joint.q[0], joint.q[1], joint.q[2],
          joint.q[3], joint.q[4], joint.q[5]};
}

std::array<double, 6> ToArray(const ArmMotion::CartesianTwist& twist) {
  return {twist.linear.x,  twist.linear.y,  twist.linear.z,
          twist.angular.x, twist.angular.y, twist.angular.z};
}

double CalculateError(const std::array<double, 6>& lhs,
                      const std::array<double, 6>& rhs) {
  double squared_sum = 0.0;
  for (int i = 0; i < 6; ++i) {
    const double diff = lhs[i] - rhs[i];
    squared_sum += diff * diff;
  }
  return std::sqrt(squared_sum);
}

ArmMotion::Joint6Dof IntegratePosition(const ArmMotion::Joint6Dof& q,
                                         const ArmMotion::Joint6Dof& dq,
                                         const ArmMotion::Joint6Dof& ddq,
                                         double dt) {
  ArmMotion::Joint6Dof q_next;
  for (int i = 0; i < 6; ++i) {
    q_next.q[i] =
        q.q[i] + dq.q[i] * dt + 0.5 * ddq.q[i] * dt * dt;
  }
  return q_next;
}

ArmMotion::Joint6Dof IntegrateVelocity(const ArmMotion::Joint6Dof& dq,
                                         const ArmMotion::Joint6Dof& ddq,
                                         double dt) {
  ArmMotion::Joint6Dof dq_next;
  for (int i = 0; i < 6; ++i) {
    dq_next.q[i] = dq.q[i] + ddq.q[i] * dt;
  }
  return dq_next;
}

ArmMotion::CartesianTwist DifferentiateTwist(
    const ArmMotion::CartesianTwist& current,
    const ArmMotion::CartesianTwist& previous, double dt) {
  return ArmMotion::CartesianTwist(
      (current.linear.x - previous.linear.x) / dt,
      (current.linear.y - previous.linear.y) / dt,
      (current.linear.z - previous.linear.z) / dt,
      (current.angular.x - previous.angular.x) / dt,
      (current.angular.y - previous.angular.y) / dt,
      (current.angular.z - previous.angular.z) / dt);
}

void PrintTwist(const char* label, const ArmMotion::CartesianTwist& twist) {
  std::cout << label << " linear = [" << twist.linear.x << ", "
            << twist.linear.y << ", " << twist.linear.z << "]"
            << ", angular = [" << twist.angular.x << ", " << twist.angular.y
            << ", " << twist.angular.z << "]" << std::endl;
}

void PrintJoint(const char* label, const ArmMotion::Joint6Dof& joint) {
  std::cout << label << " [" << joint.q[0] << ", " << joint.q[1] << ", "
            << joint.q[2] << ", " << joint.q[3] << ", "
            << joint.q[4] << ", " << joint.q[5] << "]" << std::endl;
}

}  // namespace

int main() {
  constexpr double kDt = 1e-3;
  constexpr double kDamping = 1e-6;
  constexpr double kVelocityErrorThreshold = 1e-6;
  constexpr double kAccelInverseErrorThreshold = 1e-5;
  constexpr double kAccelFiniteDiffErrorThreshold = 5e-3;

  const ArmMotion::MDHParams mdh_params{
      .alpha = {0.0, -M_PI_2, 0.0, -M_PI_2, M_PI_2, -M_PI_2},
      .a = {0.0, 0.1, 0.4, 0.35, 0.0, 0.0},
      .d = {0.2, 0.0, 0.0, 0.05, 0.0, 0.1},
      .theta = {0.0, -M_PI_2, 0.0, 0.0, 0.0, 0.0}};
  Jacobian6Dof jac(mdh_params);
  const ArmMotion::Vector3d p_tool{0.0, 0.0, 0.0};

  const ArmMotion::Joint6Dof q_current{0.35, -0.65, 0.90, -0.40, 0.55, 0.25};
  const ArmMotion::Joint6Dof dq{0.40, -0.25, 0.18, 0.12, -0.08, 0.15};
  const ArmMotion::Joint6Dof ddq{0.30, -0.18, 0.12, -0.10, 0.06, 0.08};

  jac.Update(q_current, 0.0, p_tool);

  ArmMotion::CartesianTwist v_current;
  jac.ForwardVelocity(dq, v_current);

  ArmMotion::Joint6Dof dq_reconstructed;
  jac.InverseVelocity(v_current, dq_reconstructed, kDamping);

  const ArmMotion::Joint6Dof q_next =
      IntegratePosition(q_current, dq, ddq, kDt);
  const ArmMotion::Joint6Dof dq_next = IntegrateVelocity(dq, ddq, kDt);

  jac.Update(q_next, kDt, p_tool);

  ArmMotion::CartesianTwist v_next;
  jac.ForwardVelocity(dq_next, v_next);

  ArmMotion::CartesianTwist a_predicted;
  jac.ForwardAccel(ddq, a_predicted);

  ArmMotion::Joint6Dof ddq_reconstructed;
  jac.InverseAccel(a_predicted, ddq_reconstructed, kDamping);

  const ArmMotion::CartesianTwist a_finite_diff =
      DifferentiateTwist(v_next, v_current, kDt);

  const double velocity_inverse_error =
      CalculateError(ToArray(dq), ToArray(dq_reconstructed));
  const double accel_inverse_error =
      CalculateError(ToArray(ddq), ToArray(ddq_reconstructed));
  const double accel_finite_diff_error =
      CalculateError(ToArray(a_predicted), ToArray(a_finite_diff));

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "--- Jacobian velocity test ---" << std::endl;
  PrintJoint("input dq      =", dq);
  PrintTwist("J * dq        =", v_current);
  PrintJoint("J^-1 * (Jdq)  =", dq_reconstructed);
  std::cout << "velocity inverse error = " << velocity_inverse_error
            << std::endl;

  std::cout << "\n--- Jacobian acceleration test ---" << std::endl;
  PrintJoint("input ddq     =", ddq);
  PrintTwist("J*ddq+Jdot*dq =", a_predicted);
  PrintTwist("finite diff   =", a_finite_diff);
  PrintJoint("J^-1 * accel  =", ddq_reconstructed);
  std::cout << "accel inverse error    = " << accel_inverse_error << std::endl;
  std::cout << "accel finite diff error = " << accel_finite_diff_error
            << std::endl;

  const bool velocity_ok = velocity_inverse_error < kVelocityErrorThreshold;
  const bool accel_inverse_ok =
      accel_inverse_error < kAccelInverseErrorThreshold;
  const bool accel_finite_diff_ok =
      accel_finite_diff_error < kAccelFiniteDiffErrorThreshold;

  std::cout << "\nresult: "
            << ((velocity_ok && accel_inverse_ok && accel_finite_diff_ok)
                    ? "PASS"
                    : "FAIL")
            << std::endl;

  return (velocity_ok && accel_inverse_ok && accel_finite_diff_ok) ? 0 : 1;
}