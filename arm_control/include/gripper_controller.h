#pragma once

#include <condition_variable>
#include <mutex>
#include <queue>

class GripperController {
 public:
  struct GripperCommand {
    double position;
    double velocity;
    double effort;
    double stiffness;
    double damping;
  };
  GripperController(double stiffness, double damping) : stiffness_(stiffness), damping_(damping) {}

  void SetGripperState(double position, double velocity, double effort);
  int GetGripperCommand(GripperCommand& command);
  void SetMaxRange(double max_range) { max_range_ = max_range; }
  void WaitForCompletion();
  bool PosControl(double position, double velocity, double accel, double effort);
  // bool TorqueControl(double effort, double ramp_rate = 3.0, double pos_limit = 0.0);

 private:
  double stiffness_{0.0};
  double damping_{0.0};
  double max_range_{0.0};
  double sample_frequency_ = 100.0;  // Hz

  // current state
  double current_position_{0.0};
  double current_velocity_{0.0};
  double current_effort_{0.0};

  std::mutex state_mutex_;
  std::queue<GripperCommand> command_queue_;
  std::condition_variable command_cv_;
};
