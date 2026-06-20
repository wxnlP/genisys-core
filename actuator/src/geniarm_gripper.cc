#include "geniarm_gripper.h"


void GeniArmGripper::Init() {
  state_update_thread_ = std::jthread([this] (std::stop_token stoken) {
    while (!stoken.stop_requested()) {
      motor_driver_.RequestState();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        DmMotorDriver::MotorState state;
        motor_driver_.GetLatestState(state);
        last_gripper_state_.position = state.pos;
        last_gripper_state_.velocity = state.vel;
        last_gripper_state_.effort = state.torque;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });
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
  std::lock_guard<std::mutex> lock(state_mutex_);
  state.position = last_gripper_state_.position;
  state.velocity = last_gripper_state_.velocity;
  state.effort = last_gripper_state_.effort;
  return true;
}