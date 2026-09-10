#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>

#include "actuator_manager.h"
#include "arm_logger.h"
#include "arm_manager.h"
#include "gripper_controller.h"

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

int main() {
  std::signal(SIGINT, SignalHandler);

  ActuatorManager actuator_manager("geniarm_actuator_cfg.yaml");
  ArmManager arm_manager("geniarm_body_cfg.yaml");

  auto gripper_actuator = actuator_manager.GetGripperActuator();
  auto gripper = arm_manager.GetGripperController();

  if (!gripper_actuator || !gripper) {
    ARM_LOG_ERROR("配置文件中没有 gripper 段，本示例需要带夹爪的配置（如 geniarm_actuator_gripper_cfg.yaml）");
    return 1;
  }

  gripper_actuator->Enable();

  // 控制回路：读反馈 → 取指令 → 下发
  std::jthread loop([&](std::stop_token stoken) {
    auto next = std::chrono::steady_clock::now();
    while (!stoken.stop_requested() && g_running) {
      GripperCtrlBase::GripperState state;
      if (gripper_actuator->ReceiveState(state)) {
        gripper->SetGripperState(state.position, state.velocity, state.effort);
      }
      GripperController::GripperCommand cmd;
      if (gripper->GetGripperCommand(cmd) == 0) {
        GripperCtrlBase::GripperCommand out;
        out.position = cmd.position;
        out.velocity = cmd.velocity;
        out.effort = cmd.effort;
        out.stiffness = cmd.stiffness;
        out.damping = cmd.damping;
        gripper_actuator->SendCommand(out);
      }
      next += std::chrono::milliseconds(5);  // 200 Hz
      std::this_thread::sleep_until(next);
    }
  });

  // 状态监视
  std::jthread monitor([&](std::stop_token stoken) {
    while (!stoken.stop_requested() && g_running) {
      GripperCtrlBase::GripperState state;
      if (gripper_actuator->ReceiveState(state)) {
        ARM_LOG_INFO("夹爪位置: {:.4f}, 速度: {:.4f}, 力矩: {:.4f}",
                     state.position, state.velocity, state.effort);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  ARM_LOG_INFO("闭合夹爪");
  gripper->PosControl(0.4, 0.5, 1.0, 0.0);
  gripper->WaitForCompletion();
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  ARM_LOG_INFO("张开夹爪");
  gripper->PosControl(0.0, 0.5, 1.0, 0.0);
  gripper->WaitForCompletion();

  while (g_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ARM_LOG_INFO("回收线程中......");
  monitor.request_stop();
  loop.request_stop();
  loop.join();
  monitor.join();

  ARM_LOG_INFO("失能夹爪");
  gripper_actuator->Disable();
  return 0;
}
