#include "geniarm_ctrl.h"
#include "arm_logger.h"

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
          latest_joint_state_.position.q[i] = state.pos;
          latest_joint_state_.velocity.q[i] = state.vel;
          latest_joint_state_.effort.q[i] = state.torque;
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

bool GeniArmCtrl::SetJointState(const ArmMotion::JointCommand_t& joint_cmd) {
  for (int i = 0; i < motor_driver_.size(); ++i) {
    DmMotorDriver::CtrlCmd cmd(joint_cmd.position.q[i], joint_cmd.velocity.q[i], joint_cmd.effort.q[i], joint_cmd.stiffness.q[i], joint_cmd.damping.q[i]);
    if (motor_driver_[i].MoveCtrl(cmd) == -1) {
      return false;
    }
  }
  return true;
}

bool GeniArmCtrl::GetJointState(ArmMotion::JointState_t& joint_state) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  joint_state = latest_joint_state_;
  return true;
}