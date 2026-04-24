#pragma once

#include "arm_ctrl_base.h"
#include "serial/serial.h"

class S6h4dCtrl : public ArmCtrlBase {
 public:
  S6h4dCtrl() = default;
  S6h4dCtrl(serial::Serial& serial) : serial_(serial) {};
  ~S6h4dCtrl() override = default;

  bool Init() override;
  bool SetJointState(const ArmMotion::JointState_t& joint_cmd) override;
  bool GetJointState(ArmMotion::JointState_t& joint_state) override;

 private:
  void Float2Bytes(const ArmMotion::JointState_t& joint, uint8_t* buffer);
  void Bytes2Float(const uint8_t* bytes, ArmMotion::Joint6Dof_t& joint);

  serial::Serial& serial_;
};