#pragma once

#include "arm_type.h"

class ArmCtrlBase {
 public:
  ArmCtrlBase() = default;
  virtual ~ArmCtrlBase() = default;

  virtual bool Init() = 0;
  virtual bool SetJointState(const ArmMotion::JointState_t& joint_cmd) = 0;
  virtual bool GetJointState(ArmMotion::Joint6Dof_t& joint_state) = 0;
};