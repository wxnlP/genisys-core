#include "geniarm_gripper.h"
#include <chrono>


void GeniArmGripper::Init() {
}

void GeniArmGripper::Enable() {
  motor_driver_.Enable(true);
}

void GeniArmGripper::Disable() {
  motor_driver_.Enable(false);
}


bool GeniArmGripper::SendCommand(const GripperCommand& command) {
  DmMotorDriver::CtrlCmd cmd;
  cmd.pos = command.position;
  cmd.vel = command.velocity;
  cmd.torque = command.effort;
  cmd.stiffness = command.stiffness;
  cmd.damping = command.damping;
  if (motor_driver_.MoveCtrl(cmd) == -1) {
    return false;
  }
  return true;
}

bool GeniArmGripper::ReceiveState(GripperState& state) {
  static auto last_request_time = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  if (now - last_request_time > std::chrono::milliseconds(100)) {
    motor_driver_.RequestState();
    last_request_time = now;
  }

  DmMotorDriver::MotorState motor_state;
  if (motor_driver_.GetLatestState(motor_state) == 0) {
    state.position = motor_state.pos;
    state.velocity = motor_state.vel;
    state.effort = motor_state.torque;
  }
  return true;
}