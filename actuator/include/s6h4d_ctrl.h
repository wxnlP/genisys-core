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
  bool SetJointState(const ArmCtrlBase::JointCommand& joint_cmd) override;
  bool GetJointState(ArmCtrlBase::JointState& joint_state) override;

 private:
  void Float2Bytes(const ArmCtrlBase::JointCommand& joint, uint8_t* buffer);
  void Bytes2Float(const uint8_t* bytes, ArmCtrlBase::JointState& joint);

  serial::Serial& serial_;
  std::jthread joint_state_thread_;
  std::mutex joint_state_mutex_;
  ArmCtrlBase::JointState last_joint_state_{};
};