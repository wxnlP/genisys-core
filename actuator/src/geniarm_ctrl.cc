#include "geniarm_ctrl.h"
#include <chrono>


GeniArmCtrl::GeniArmCtrl(std::vector<DmMotorDriver>& motor_driver) : motor_driver_(motor_driver) {}

GeniArmCtrl::~GeniArmCtrl() = default;

bool GeniArmCtrl::Init() {
  return true;
}

void GeniArmCtrl::Enable() {
  for (auto& motor : motor_driver_) {
    motor.Enable(true);
  }
}

void GeniArmCtrl::Disable() {
  for (auto& motor : motor_driver_) {
    motor.Enable(false);
  }
}

bool GeniArmCtrl::SetJointState(const ArmCtrlBase::JointCommand& joint_cmd) {
  for (int i = 0; i < motor_driver_.size(); ++i) {
    DmMotorDriver::CtrlCmd cmd(joint_cmd.position[i], joint_cmd.velocity[i], joint_cmd.effort[i], joint_cmd.stiffness[i], joint_cmd.damping[i]);
    if (motor_driver_[i].MoveCtrl(cmd) == -1) {
      return false;
    }
  }
  return true;
}

bool GeniArmCtrl::GetJointState(ArmCtrlBase::JointState& joint_state) {
  static auto last_request_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (now - last_request_time > std::chrono::milliseconds(100)) {
    for (auto& motor : motor_driver_) {
      motor.RequestState();
    }
    last_request_time = now;
  }

  for (int i = 0; i < motor_driver_.size(); ++i) {
    DmMotorDriver::MotorState state;
    if (motor_driver_[i].GetLatestState(state) == 0) {
      joint_state.position[i] = state.pos;
      joint_state.velocity[i] = state.vel;
      joint_state.effort[i] = state.torque;
    }
  }
  return true;
}

bool GeniArmCtrl::SetCtrlMode(ArmCtrlBase::CtrlMode mode) {
  DmMotorDriver::CtrlMode dm_mode;
  switch (mode) {
    case ArmCtrlBase::CtrlMode::POS:
      dm_mode = DmMotorDriver::CtrlMode::POS;
      break;
    case ArmCtrlBase::CtrlMode::VEL:
      dm_mode = DmMotorDriver::CtrlMode::VEL;
      break;
    default:
      dm_mode = DmMotorDriver::CtrlMode::MIT;
      break;
  }
  for (auto& motor : motor_driver_) {
    if (motor.SetCtrlMode(dm_mode, false) == -1) {
      return false;
    }
  }
  return true;
}
