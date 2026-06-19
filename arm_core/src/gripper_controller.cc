#include "gripper_controller.h"

#include "lspb_time_scaler.h"
#include "arm_logger.h"
#include <chrono>
#include <cmath>
#include <thread>
#include <condition_variable>

void GripperController::Enable() {
  gripper_.Enable();
}

void GripperController::Disable() {
  gripper_.Disable();
}

bool GripperController::SetPosition(double position, double velocity,
                                    double accel, double effort) {
  const double max_range = gripper_.GetMaxRange();

  double current_pos{};
  double current_vel{};
  double current_effort{};
  if (!gripper_.ReceiveState(current_pos, current_vel, current_effort)) {
    return false;
  }

  ARM_LOG_INFO("Current State: {} {} {}", current_pos, current_vel, current_effort);

  double target_pos = position * max_range;

  double distance = target_pos - current_pos;
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

    const double pos_phys = current_pos + state.s * distance;
    const double vel_phys = state.sd * distance;

    if (!gripper_.SendCommand(pos_phys, vel_phys, effort,
                              gripper_.GetStiffness(), gripper_.GetDamping())) {
      return false;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(kDefaultDt));
    t += kDefaultDt;
  }

  return true;
}

bool GripperController::SetTorque(double effort, double ramp_rate) {
  double current_pos{}, current_vel{}, current_effort{};
  if (!gripper_.ReceiveState(current_pos, current_vel, current_effort)) {
    ARM_LOG_ERROR("SetTorque: ReceiveState failed");
    return false;
  }

  ARM_LOG_INFO("SetTorque: current_effort={:.3f}Nm, target={:.3f}Nm, ramp={:.1f}Nm/s",
               current_effort, effort, ramp_rate);

  const double delta = effort - current_effort;
  const double duration = std::abs(delta) / ramp_rate;
  double t = 0.0;

  constexpr double kKd = 0.5;

  std::atomic<bool> pos_limited{false};
  std::condition_variable cv;
  std::mutex cv_mutex;

  std::jthread pos_limit_thread([this, &cv, &pos_limited, &effort](std::stop_token stoken) {
    while (!stoken.stop_requested()) {
      double now_pos{}, now_vel{}, now_effort{};
      gripper_.ReceiveState(now_pos, now_vel, now_effort);
      if (std::abs(now_pos) > std::abs(gripper_.GetMaxRange()) + 0.5) {
        pos_limited = true;
        cv.notify_all();
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (pos_limited && !stoken.stop_requested()) {
      ARM_LOG_WARN("Triggered pos limit");
      constexpr double kBrakeRate = 10.0;
      const double brake_dur = std::abs(effort) / kBrakeRate;
      double t_brake = 0.0;

      while (t_brake <= brake_dur + 1e-7) {
        if (t_brake > brake_dur) { t_brake = brake_dur; }
        const double cmd_effort = effort * (1.0 - t_brake / brake_dur);

        gripper_.SendCommand(0.0, 0.0, cmd_effort, 0.0, kKd);
        std::this_thread::sleep_for(std::chrono::duration<double>(kDefaultDt));
        t_brake += kDefaultDt;
      }

      gripper_.SendCommand(0.0, 0.0, 0.0, 0.0, kKd);
      ARM_LOG_INFO("SetTorque: SAFE, torque released");
    }
  });

  while (t <= duration + 1e-7 && (!pos_limited)) {
    if (t > duration) { t = duration; }

    const double alpha = (duration > 1e-9) ? (t / duration) : 1.0;
    const double cmd_effort = current_effort + delta * alpha;

    if (!gripper_.SendCommand(0.0, 0.0, cmd_effort, 0.0, kKd)) {
      ARM_LOG_ERROR("SetTorque: SendCommand failed at t={:.3f}s, "
                    "effort={:.3f}Nm",
                    t, cmd_effort);
      return false;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(kDefaultDt));
    t += kDefaultDt;
  }

  if (!pos_limited) {
    ARM_LOG_INFO("SetTorque: Target torque reached. Waiting for pos limit (max 3s)...");

    std::unique_lock<std::mutex> lk(cv_mutex);
    bool cv_ret = cv.wait_for(lk, std::chrono::seconds(3), [&pos_limited]() {
      return pos_limited.load();
    });

    if (!cv_ret) {
      ARM_LOG_ERROR("SetTorque: Timeout! Position limit not reached within 3s. Exit with error.");

      gripper_.SendCommand(0.0, 0.0, 0.0, 0.0, kKd);

      pos_limit_thread.request_stop();
      pos_limit_thread.join();
      return false;
    }
  }

  ARM_LOG_INFO("SetTorque: Pos limit detected successfully. Waiting for brake release...");

  pos_limit_thread.join();

  ARM_LOG_INFO("SetTorque: Task completed successfully.");

  return true;
}

bool GripperController::GetPosition(double& position, double& effort) {
  double position_phys = 0.0;
  double velocity_phys = 0.0;
  if (!gripper_.ReceiveState(position_phys, velocity_phys, effort)) {
    return false;
  }
  position = position_phys / gripper_.GetMaxRange();
  return true;
}
