#include "gripper_controller.h"

#include "lspb_time_scaler.h"
#include "arm_logger.h"
#include <cmath>


void GripperController::SetGripperState(double position, double velocity, double effort) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  current_position_ = position;
  current_velocity_ = velocity;
  current_effort_ = effort;
}

int GripperController::GetGripperCommand(GripperCommand& command) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (command_queue_.empty()) {
    return -1;
  }
  command = command_queue_.front();
  command_queue_.pop();
  if (command_queue_.empty()) {
    command_cv_.notify_all();
  }
  return 0;
}

void GripperController::WaitForCompletion() {
  std::unique_lock<std::mutex> lock(state_mutex_);
  command_cv_.wait(lock, [this] { return command_queue_.empty(); });
}

bool GripperController::PosControl(double position, double velocity, double accel, double effort) {
  double current_position;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_position = current_position_;
    ARM_LOG_INFO("Current State: {} {} {}", current_position_, current_velocity_, current_effort_);
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

    // TODO: 队列
    GripperCommand cmd;
    cmd.position = current_position + state.s * distance;
    cmd.velocity = state.sd * distance;
    cmd.effort = effort;
    cmd.stiffness = stiffness_;
    cmd.damping = damping_;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      command_queue_.push(cmd);
    }

    t += 1.0 / sample_frequency_;
  }

  return true;
}
