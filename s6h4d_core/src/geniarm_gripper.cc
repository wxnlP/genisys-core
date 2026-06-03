#include "geniarm_gripper.h"

double GeniArmGripper::GetMaxRange() const {
  return kMaxRange;
}

void GeniArmGripper::Enable() {
  motor_driver_.Enable(true);
}

void GeniArmGripper::Disable() {
  motor_driver_.Enable(false);
}


bool GeniArmGripper::SendCommand(double position, double velocity, double effort, double stiffness, double damping) {
  DmMotorDriver::CtrlCmd cmd;
  cmd.pos = position;
  cmd.vel = velocity;
  cmd.torque = effort;
  cmd.stiffness = stiffness;
  cmd.damping = damping;
  if (motor_driver_.MoveCtrl(cmd) == -1) {
    return false;
  }
  return true;
}

bool GeniArmGripper::ReceiveState(double& position, double& velocity, double& effort) {
  DmMotorDriver::MotorState state;
  if (motor_driver_.GetLatestState(state) == -1) {
    return false;
  }
  position = state.pos;
  velocity = state.vel;
  velocity = state.torque;
  return true;
}