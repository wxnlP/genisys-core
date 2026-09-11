#pragma once

#include <mutex>
#include <optional>
#include "spsc_ring_buffer.h"

class GripperController {
 public:
  struct GripperCommand {
    double position{};
    double velocity{};
    double effort{};
    double stiffness{};
    double damping{};
  };
  GripperController(double stiffness, double damping) : stiffness_(stiffness), damping_(damping) {}

  void SetGripperState(double position, double velocity, double effort);
  int GetCurrentPosition(double& position);
  int GetGripperCommand(GripperCommand& command);
  void SetMaxRange(double max_range) { max_range_ = max_range; }
  void SetSampleFrequency(double hz) { sample_frequency_ = hz; }
  double GetSampleFrequency() const { return sample_frequency_; }
  void WaitForCompletion();
  bool PosControl(double position, double velocity, double accel, double effort);
  // bool TorqueControl(double effort, double ramp_rate = 3.0, double pos_limit = 0.0);

 private:
  double stiffness_{0.0};
  double damping_{0.0};
  double max_range_{0.0};
  double sample_frequency_ = 100.0;  // Hz

  // current state
  std::optional<double> current_position_{};
  std::optional<double> current_velocity_{};
  std::optional<double> current_effort_{};

  std::mutex state_mutex_;
  SPSCRingBuffer<GripperCommand> command_queue_{4096};
};
