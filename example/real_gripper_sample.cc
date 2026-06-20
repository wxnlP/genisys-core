#include <chrono>
#include <thread>
#include "actuator_manager.h"
#include "arm_logger.h"
#include "gripper_controller.h"

int main() {
  ActuatorManager actuator("actuator_config.yaml");
  auto gripper = actuator.GetGripperController();

  GripperController gripper_ctrl(8.0, 1.0);  // stiffness, damping
  gripper_ctrl.SetMaxRange(1.95);

  gripper->Init();
  gripper->Enable();

  std::jthread receive_thread([&gripper]() {
    GripperCtrlBase::GripperState state;
    while (true) {
      if (gripper->ReceiveState(state)) {
        ARM_LOG_INFO("==================Motor State=====================");
        ARM_LOG_INFO("pos: {} vel: {} torque: {}",
                     state.position, state.velocity, state.effort);
        ARM_LOG_INFO("==================================================");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  /* 位置速度控制 */
  // gripper_ctrl.PosControl(1.0, 0.5, 1.0, 0.0);
  // ARM_LOG_INFO("PosControl: done");
  // std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  // gripper_ctrl.PosControl(0.0, 0.5, 1.0, 0.0);

  /* 力控 — 力矩控制接口暂未实现，先注释 */
  // ARM_LOG_INFO("=== Test 1: -3Nm torque ===");
  // gripper_ctrl.TorqueControl(-3.0, 1.0);
  // std::this_thread::sleep_for(std::chrono::milliseconds(3000));

  ARM_LOG_INFO("夹爪位置控制测试");
  gripper_ctrl.PosControl(0.4, 0.5, 1.0, 0.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  ARM_LOG_INFO("回归零点......");
  gripper_ctrl.PosControl(0.0, 0.5, 1.0, 0.0);

  gripper->Disable();
  return 0;
}