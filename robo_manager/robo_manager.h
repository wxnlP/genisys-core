#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include "actuator_manager.h"
#include "arm_manager.h"
#include "arm_controller.h"
#include "arm_ctrl_base.h"
#include "gripper_controller.h"
#include "gripper_ctrl_base.h"

class RoboManager {
 public:
  enum class State {
    DISABLED,
    IDLE,
    MOVING,
    PAUSED,
    ERROR,
  };

  explicit RoboManager(const std::string& config_yaml);
  ~RoboManager();

  void Start();
  void Stop();
  void EmergencyStop();

  State GetState() const;
  bool IsMoving() const;

  void Enable();
  void Disable();

  void SetCycleFrequency(double hz);

  std::shared_ptr<ArmController> GetArmController();
  std::shared_ptr<GripperController> GetGripperController();

 private:
  void ControlLoop();
  void WaitNextCycle(std::chrono::steady_clock::time_point t_start);

  // 管理器
  std::shared_ptr<ActuatorManager> actuator_mgr_;
  std::shared_ptr<ArmManager> arm_mgr_;

  // 驱动层
  std::shared_ptr<ArmCtrlBase> actuator_;
  std::shared_ptr<GripperCtrlBase> gripper_actuator_;

  // 算法层
  std::shared_ptr<ArmController> arm_controller_;
  std::shared_ptr<GripperController> gripper_controller_;

  std::atomic<State> state_{State::DISABLED};
  std::atomic<bool> stop_requested_{false};
  std::unique_ptr<std::jthread> control_thread_;

  double dt_{0.005};        // 默认 200 Hz
  int overrun_counter_{0};  // 连续掉帧计数
};
