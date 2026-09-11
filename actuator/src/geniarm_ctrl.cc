#include "geniarm_ctrl.h"
#include <chrono>
#include <cmath>
#include <thread>

namespace {

// 标零验证参数
constexpr double kZeroPosTol = 0.001;  // rad，位置容差
constexpr int kZeroTimeoutMs = 800;    // 验证超时
constexpr int kZeroPollMs = 20;        // 轮询间隔

}  // namespace

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
    if (motor_driver_[i].GetLatestState(state) != 0) {
      return false;
    }
    joint_state.position[i] = state.pos;
    joint_state.velocity[i] = state.vel;
    joint_state.effort[i] = state.torque;
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

uint32_t GeniArmCtrl::CalibrateZero() {
  // set zero position for all motors
  for (auto& motor : motor_driver_) {
    motor.SetZeroPos();
  }

  const size_t joint_num = motor_driver_.size();
  const uint32_t target_mask = (1U << joint_num) - 1;
  uint32_t verified_mask = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(kZeroTimeoutMs);

  // Verify all joints
  while (verified_mask != target_mask && std::chrono::steady_clock::now() < deadline) {
    // 先给尚未通过验证的关节统一发一次状态请求（GetLatestState 读的是后台
    // Sample 线程缓存的帧，不主动索取就可能一直读到标零前的旧值）
    for (size_t i = 0; i < joint_num; ++i) {
      if (!(verified_mask & (1U << i))) {
        motor_driver_[i].RequestState();
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(kZeroPollMs));  // 等一轮反馈

    // 再统一读回并记录本轮通过的关节
    for (size_t i = 0; i < joint_num; ++i) {
      if (verified_mask & (1U << i)) {
        continue;
      }
      DmMotorDriver::MotorState state;
      if (motor_driver_[i].GetLatestState(state) == 0 && std::fabs(state.pos) <= kZeroPosTol) {
        verified_mask |= (1U << i);
      }
    }
  }

  return verified_mask;
}
