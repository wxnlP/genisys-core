#pragma once

#include "arm_ctrl_base.h"
#include "serial/serial.h"
#include <thread>
#include <mutex>

class S6h4dCtrl : public ArmCtrlBase {
 public:
  S6h4dCtrl() = default;
  S6h4dCtrl(serial::Serial& serial) : serial_(serial) {};
  ~S6h4dCtrl() override = default;

  bool Init() override;
  bool SetJointState(const ArmMotion::JointCommand_t& joint_cmd) override;
  bool GetJointState(ArmMotion::JointState_t& joint_state) override;

 private:
  void Float2Bytes(const ArmMotion::JointCommand_t& joint, uint8_t* buffer);
  void Bytes2Float(const uint8_t* bytes, ArmMotion::Joint6Dof_t& joint);

  serial::Serial& serial_;
  std::jthread joint_state_thread_;
  std::mutex joint_state_mutex_;
  ArmMotion::JointState_t last_joint_state_{};
};