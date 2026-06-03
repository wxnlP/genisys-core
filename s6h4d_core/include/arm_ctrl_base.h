#pragma once

#include "arm_type.h"

class ArmCtrlBase {
 public:
  ArmCtrlBase() = default;
  virtual ~ArmCtrlBase() = default;

  virtual bool Init() { return true; };
  virtual void Enable() { return; };
  virtual void Disable() { return; };
  virtual bool SetJointState(const ArmMotion::JointCommand_t& joint_cmd) = 0;
  virtual bool GetJointState(ArmMotion::JointState_t& joint_state) = 0;
};