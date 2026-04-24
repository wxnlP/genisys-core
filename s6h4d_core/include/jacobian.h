#pragma once

#include <memory>

#include "arm_type.h"

class Jacobian6Dof {
 public:
  explicit Jacobian6Dof(const ArmMotion::MDHConfig_t& mdh_config);
  ~Jacobian6Dof();

  void Update(const ArmMotion::Joint6Dof_t& inputJoint, double dt);
  bool ForwardVelocity(const ArmMotion::Joint6Dof_t& inputVel,
                       ArmMotion::CartesianTwist_t& outputVel);
  bool InverseVelocity(const ArmMotion::CartesianTwist_t& inputVel,
                       ArmMotion::Joint6Dof_t& outputVel, double damping);
  bool ForwardAccel(const ArmMotion::Joint6Dof_t& inputAcc,
                    ArmMotion::CartesianTwist_t& outputAcc);
  bool InverseAccel(const ArmMotion::CartesianTwist_t& inputAcc,
                    ArmMotion::Joint6Dof_t& outputAcc, double damping);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  // 记录机械角度偏移
  double jointOffset_[6];
};