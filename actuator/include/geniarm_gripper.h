#pragma once

#include <thread>
#include <mutex>
#include "dm_motor_driver.h"
#include "gripper_ctrl_base.h"


class GeniArmGripper : public GripperCtrlBase {
 public:
  GeniArmGripper(DmMotorDriver& motor_driver, double stiffness, double damping)
      : GripperCtrlBase(stiffness, damping), motor_driver_(motor_driver) {}
  ~GeniArmGripper() override = default;

  void Init() override;
  void Enable() override;
  void Disable() override;
  bool ReceiveState(double& position, double& velocity, double& effort) override;

 private:
  bool SendCommand(double position, double velocity, double effort, double stiffness, double damping) override;

  static constexpr double kMaxRange = 1.9;  // 弧度或米，取决于使用场景
  DmMotorDriver& motor_driver_;
  DmMotorDriver::MotorState latest_state_{};
  std::mutex state_mutex_;
  std::jthread state_update_thread_;
};