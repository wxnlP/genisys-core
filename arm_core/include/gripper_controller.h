#pragma once

#include "gripper_ctrl_base.h"

class GripperController {
 public:
  explicit GripperController(GripperCtrlBase& gripper) : gripper_(gripper) {}

  bool SetPosition(double position, double velocity, double accel, double effort);
  bool SetTorque(double effort, double ramp_rate = 3.0);
  bool GetPosition(double& position, double& effort);

 private:
  GripperCtrlBase& gripper_;
  static constexpr double kDefaultDt = 0.01;
};
