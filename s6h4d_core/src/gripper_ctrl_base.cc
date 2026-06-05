#include "gripper_ctrl_base.h"

#include "lspb_time_scaler.h"
#include "arm_logger.h"
#include <chrono>
#include <cmath>
#include <thread>

bool GripperCtrlBase::SetPosition(double position, double velocity,
                                  double accel, double effort) {
  const double max_range = max_range_;

  // 读取当前物理位置，转归一化
  double current_pos{};
  double current_vel{};
  double current_effort{};
  if (!ReceiveState(current_pos, current_vel, current_effort)) {
    return false;
  }

  ARM_LOG_INFO("Current State: {} {} {}", current_pos, current_vel, current_effort);

  double target_pos = position * max_range;

  // 已在目标位置
  double distance = target_pos - current_pos;
  if (std::abs(distance) < 1e-6) {
    return true;
  }

  // ── 物理值 → 归一化（同 MoveJ 的 BuildLspbConfigFromPhysicalVA）──
  const double abs_dist = std::abs(distance);
  if (velocity <= 0.0 || accel <= 0.0 || abs_dist < 1e-9) {
    return false;
  }

  LSPBTimeScaler scaler;
  TimeScalerBase::PlanConfig config;
  config.v_max = velocity / abs_dist;
  config.a_max = accel / abs_dist;
  if (!scaler.Plan(config)) {
    return false;
  }

  const double duration = scaler.GetDuration();
  double t = 0.0;
  while (t <= duration + 1e-7) {
    if (t > duration) {
      t = duration;
    }

    TimeScalerBase::State state;
    if (!scaler.Evaluate(t, state)) {
      return false;
    }

    // 归一化 → 物理值
    const double pos_phys = current_pos + state.s * distance;
    const double vel_phys = state.sd * distance;
    // const double accel_phys = state.sdd * distance;

    if (!SendCommand(pos_phys, vel_phys, effort, motor_stiffness_, motor_damping_)) {
      return false;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(kDefaultDt));
    t += kDefaultDt;
  }

  return true;
}

bool GripperCtrlBase::GetPosition(double& position, double& effort) {
  double position_phys = 0.0;
  double velocity_phys = 0.0;
  if (!ReceiveState(position_phys, velocity_phys, effort)) {
    return false;
  }
  position = position_phys / max_range_;
  return true;
}
