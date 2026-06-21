#include "geniarm_ctrl.h"


GeniArmCtrl::GeniArmCtrl(std::vector<DmMotorDriver>& motor_driver) : motor_driver_(motor_driver) {}

GeniArmCtrl::~GeniArmCtrl() = default;

bool GeniArmCtrl::Init() {
  state_update_thread_ = std::jthread([this](std::stop_token stoken) {
    while (!stoken.stop_requested()) {
      for (auto& motor : motor_driver_) {
        motor.RequestState();
      }
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (int i = 0; i < motor_driver_.size(); ++i) {
          DmMotorDriver::MotorState state;
          motor_driver_[i].GetLatestState(state);
          latest_joint_state_.position[i] = state.pos;
          latest_joint_state_.velocity[i] = state.vel;
          latest_joint_state_.effort[i] = state.torque;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }); 
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

void GeniArmCtrl::Shutdown() {
  state_update_thread_.request_stop();
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
  std::lock_guard<std::mutex> lock(state_mutex_);
  joint_state = latest_joint_state_;
  return true;
}

bool GeniArmCtrl::SetCtrlMode(DmMotorDriver::CtrlMode mode) {
  for (auto& motor : motor_driver_) {
    if (motor.SetCtrlMode(mode, false) == -1) {
      return false;
    }
  }
  return true;
}
