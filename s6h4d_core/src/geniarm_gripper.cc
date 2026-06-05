#include "geniarm_gripper.h"


void GeniArmGripper::Init() {
  state_update_thread_ = std::jthread([this] (std::stop_token stoken) {
    while (!stoken.stop_requested()) {
      motor_driver_.RequestState();
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        motor_driver_.GetLatestState(latest_state_);
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
  std::lock_guard<std::mutex> lock(state_mutex_);
  position = latest_state_.pos;
  velocity = latest_state_.vel;
  effort = latest_state_.torque;
  return true;
}