#include "geniarm_ctrl.h"

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

bool GeniArmCtrl::SendJointStateRequest() {
  for (auto& motor : motor_driver_) {
    if (!motor.RequestState()) {
      return false;
    }
  }
  return true;
}

bool GeniArmCtrl::SetJointState(const ArmMotion::JointCommand_t& joint_cmd) {
  for (int i = 0; i < motor_driver_.size(); ++i) {
    DmMotorDriver::CtrlCmd cmd(joint_cmd.velocity.q[i], joint_cmd.position.q[i], joint_cmd.effort.q[i], joint_cmd.stiffness.q[i], joint_cmd.damping.q[i]);
    if (!motor_driver_[i].MoveCtrl(cmd)) {
      return false;
    }
  }
  return true;
}

bool GeniArmCtrl::GetJointState(ArmMotion::JointState_t& joint_state) {
  for (int i = 0; i < motor_driver_.size(); ++i) {
    DmMotorDriver::MotorState state;
    if (!motor_driver_[i].GetLatestState(state)) {
      return false;
    }
    joint_state.position.q[i] = state.pos;
    joint_state.velocity.q[i] = state.vel;
    joint_state.effort.q[i] = state.torque;
  }
  return true;
}