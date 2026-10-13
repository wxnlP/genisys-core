#include "gripper_controller.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include "arm_logger.h"
#include "lspb_time_scaler.h"
#include "quintic_time_scaler.h"

void GripperController::SetGripperState(double position, double velocity, double effort) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_position_ = position;
  current_velocity_ = velocity;
  current_effort_ = effort;
}

int GripperController::GetCurrentPosition(double& position) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!current_position_.has_value()) {
    return -1;
  }
  position = *current_position_;
  return 0;
}

int GripperController::GetGripperCommand(GripperCommand& command) {
  if (command_queue_.Pop(command)) {
    return 0;
  }
  command.damping = damping_;
  command.stiffness = stiffness_;
  return -1;
}

void GripperController::WaitForCompletion() {
  while (!command_queue_.IsEmpty()) {
    std::this_thread::yield();
  }
}

bool GripperController::PosControl(double position, double velocity, double accel, double effort) {
  double current_position;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!current_position_.has_value()) {
      ARM_LOG_ERROR("[PosControl] current_position_ is not set");
      return false;
    }
    current_position = *current_position_;
  }

  double target_pos = position * max_range_;

  double distance = target_pos - current_position;
  if (std::abs(distance) < 1e-6) {
    return true;
  }

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

    GripperCommand cmd;
    cmd.position = current_position + state.s * distance;
    cmd.velocity = state.sd * distance;
    cmd.effort = effort;
    cmd.stiffness = stiffness_;
    cmd.damping = damping_;

    if (!command_queue_.Push(cmd)) {
      ARM_LOG_ERROR("[Gripper] Command queue full! Dropping trajectory point.");
    }

    t += 1.0 / sample_frequency_;
  }

  return true;
}

// ============================================================================
//  力控抓取
// ============================================================================
bool GripperController::ForceControl(const ForceControlParams& params) {
  if (params.target_torque <= 0.0 || params.target_torque > force_config_.max_target_torque ||
      params.switch_torque < 0.0 || params.approach_velocity <= 0.0 ||
      params.approach_accel <= 0.0) {
    ARM_LOG_ERROR(
        "[Gripper] ForceControl 参数非法: target={:.3f} (上限 {:.3f}), switch={:.3f}, "
        "v={:.3f}, a={:.3f}",
        params.target_torque, force_config_.max_target_torque, params.switch_torque,
        params.approach_velocity, params.approach_accel);
    return false;
  }

  force_stop_.store(false);
  const double dt = 1.0 / sample_frequency_;
  const auto tick = std::chrono::duration<double>(dt);
  const double release_pos = force_config_.min_position_ratio * max_range_;

  auto read_state = [this](double& x, double& tau) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!current_position_.has_value() || !current_effort_.has_value()) {
      return false;
    }
    x = *current_position_;
    tau = *current_effort_;
    return true;
  };
  auto push = [this](double position, double effort, double stiffness, double damping) {
    GripperCommand cmd;
    cmd.position = position;
    cmd.effort = effort;
    cmd.stiffness = stiffness;
    cmd.damping = damping;
    if (!command_queue_.Push(cmd)) {
      ARM_LOG_ERROR("[Gripper] 指令队列满");
    }
  };

  auto release = [&]() { 
    PosControl(0.0, params.approach_velocity, params.approach_accel, 0.0); 
  };

  double x = 0.0, tau = 0.0;
  if (!read_state(x, tau)) {
    ARM_LOG_ERROR("[Gripper] 力控启动失败：读不到夹爪状态");
    return false;
  }

  // ---------------- 第 1 步：合，直到碰到物体 ----------------
  const double x_start = x;
  const double distance = -x_start;  // 目标 0 = 完全闭合
  const double abs_dist = std::abs(distance);
  double duration = 0.0;
  QuinticTimeScaler scaler;
  if (abs_dist > 1e-6) {
    TimeScalerBase::PlanConfig config;
    config.v_max = params.approach_velocity / abs_dist;
    config.a_max = params.approach_accel / abs_dist;
    config.T = std::max(1.875 / config.v_max, std::sqrt(5.7735 / config.a_max));
    if (!scaler.Plan(config)) {
      ARM_LOG_ERROR("[Gripper] 接近段多项式时间标定失败");
      return false;
    }
    duration = scaler.GetDuration();
  }

  double t = 0.0;
  int confirm = 0;
  bool contact = false;
  while (read_state(x, tau)) {
    TimeScalerBase::State state{};
    scaler.Evaluate(std::min(t, duration), state);
    push(x_start + state.s * distance, 0.0, stiffness_, damping_);

    if (std::abs(tau) >= params.switch_torque) {
      if (++confirm >= force_config_.contact_ticks) {
        contact = true;
        break;
      }
    } else {
      confirm = 0;
    }

    // 空夹
    if (t >= duration || x <= release_pos) {
      release();
      ARM_LOG_WARN("[Gripper] 夹空");
      return true;
    }

    t += dt;
    std::this_thread::sleep_for(tick);
  }

  // ---------------- 第 2 步：夹住 ----------------
  const double dir = (tau >= 0.0) ? 1.0 : -1.0;
  double tau_cmd = std::min(std::abs(tau), params.target_torque);  // 从接触时的力矩无缝接管

  while (read_state(x, tau)) {
    // 位置接近闭合，是否卸力
    if (params.closed_release && x <= release_pos) {
      release();
      ARM_LOG_WARN("[Gripper] 位置 {:.4f} rad 已接近闭合，释放力矩", x);
      return true;
    }

    tau_cmd = std::min(tau_cmd + force_config_.ramp_rate * dt, params.target_torque);
    push(0.0, dir * tau_cmd, 0.0, force_config_.hold_damping);

    if (tau_cmd >= params.target_torque) {
      break;
    }
    std::this_thread::sleep_for(tick);
  }

  return true;
}

