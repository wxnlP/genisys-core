#pragma once

#include <memory>

#include "arm_type.h"

class Jacobian6Dof {
 public:
  explicit Jacobian6Dof(const ArmMotion::MDHParams& mdh_params);
  ~Jacobian6Dof();

  void Update(const ArmMotion::Joint6Dof& inputJoint, double dt);
  bool ForwardVelocity(const ArmMotion::Joint6Dof& inputVel,
                       ArmMotion::CartesianTwist& outputVel);
  bool InverseVelocity(const ArmMotion::CartesianTwist& inputVel,
                       ArmMotion::Joint6Dof& outputVel, double damping);
  bool ForwardAccel(const ArmMotion::Joint6Dof& inputAcc,
                    ArmMotion::CartesianTwist& outputAcc);
  bool InverseAccel(const ArmMotion::CartesianTwist& inputAcc,
                    ArmMotion::Joint6Dof& outputAcc, double damping);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  // 记录机械角度偏移
  double jointOffset_[6];
};