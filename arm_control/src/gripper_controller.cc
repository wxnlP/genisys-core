#include "gripper_controller.h"

#include <cmath>
#include <thread>
#include "arm_logger.h"
#include "lspb_time_scaler.h"

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
