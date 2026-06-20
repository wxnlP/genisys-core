#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include "arm_ctrl_base.h"
#include "arm_controller.h"
#include "gripper_ctrl_base.h"
#include "gripper_controller.h"

class ActuatorManager;
class ArmManager;

class RoboManager {
 public:
  enum class State {
    DISABLED,
    IDLE,
    MOVING,
    PAUSED,
    ERROR,
  };

  RoboManager(const std::string& actuator_yaml,
              const std::string& arm_yaml);
  ~RoboManager();

  // ── 生命周期 ──
  void Start();
  void Stop();
  void EmergencyStop();

  // ── 状态查询 ──
  State GetState() const;
  bool IsMoving() const;

  // ── 使能 ──
  void Enable();
  void Disable();

  // ── 控制频率 (Hz) ──
  void SetCycleFrequency(double hz);

  // ── 控制器访问（供外部调用 MoveJ / MoveL / PosControl 等）──
  ArmController* GetArmController();
  GripperController* GetGripperController();

 private:
  void ControlLoop();
  void WaitNextCycle(std::chrono::steady_clock::time_point t_start);

  // 两层管理器（持有生命周期）
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

  double dt_{0.005};  // 默认 200 Hz
};
