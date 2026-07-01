#include "geniarm_gripper.h"


void GeniArmGripper::Init() {
  state_update_thread_ = std::jthread([this] (std::stop_token stoken) {
    uint64_t cycle_count = 0;
    while (!stoken.stop_requested()) {
      // 每 100ms 发一次 RequestState
      if (cycle_count % 100 == 0) {
        motor_driver_.RequestState();
      }
      cycle_count++;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        DmMotorDriver::MotorState state;
        motor_driver_.GetLatestState(state);
        last_gripper_state_.position = state.pos;
        last_gripper_state_.velocity = state.vel;
        last_gripper_state_.effort = state.torque;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
}

void GeniArmGripper::Enable() {
  motor_driver_.Enable(true);
}

void GeniArmGripper::Disable() {
  motor_driver_.Enable(false);
}

void GeniArmGripper::Shutdown() {
  state_update_thread_.request_stop();
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