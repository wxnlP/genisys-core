#include <chrono>
#include <iomanip>
#include <iostream>
#include <thread>

#include "arm_logger.h"
#include "robo_manager.h"

namespace {

void PrintGripperState(const char* label, GripperCtrlBase& gripper) {
  double position = 0.0;
  double effort = 0.0;
  if (!gripper.GetPosition(position, effort)) {
    ARM_LOG_WARN("GetPosition failed: {}", label);
    return;
  }

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "[" << label << "] position=" << position << " ("
            << static_cast<int>(position * 100) << "%)"
            << "  effort=" << effort << " Nm\n";
}

}  // namespace

int main() {
  RoboManager mgr("geniarm_sim_config.yaml");
  auto gripper = mgr.GetGripper();

  ARM_LOG_INFO("=== Gripper Demo Start ===");

  // ── 初始状态 ──
  PrintGripperState("initial", *gripper);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // ── 第1组：慢速全开全关 3 次 ──
  ARM_LOG_INFO("--- Round 1: slow open/close x3 ---");
  for (int i = 0; i < 3; ++i) {
    ARM_LOG_INFO("  -> fully open  (1.0)");
    gripper->SetPosition(1.0, 0.01, 0.02, 0.0);
    PrintGripperState("after open", *gripper);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    ARM_LOG_INFO("  -> fully close (0.0)");
    gripper->SetPosition(0.0, 0.01, 0.02, 0.0);
    PrintGripperState("after close", *gripper);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }

  // ── 第2组：半开半合快速往复 5 次 ──
  ARM_LOG_INFO("--- Round 2: half-range fast reciprocating x5 ---");
  for (int i = 0; i < 5; ++i) {
    ARM_LOG_INFO("  -> half open (0.5)");
    gripper->SetPosition(0.5, 0.03, 0.06, 0.0);
    PrintGripperState("at 0.5", *gripper);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    ARM_LOG_INFO("  -> quarter open (0.25)");
    gripper->SetPosition(0.25, 0.03, 0.06, 0.0);
    PrintGripperState("at 0.25", *gripper);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // ── 第3组：阶梯渐开 ──
  ARM_LOG_INFO("--- Round 3: step open 0.0 -> 0.3 -> 0.6 -> 1.0 ---");
  for (double target : {0.3, 0.6, 1.0}) {
    ARM_LOG_INFO("  -> target {:.1f}", target);
    gripper->SetPosition(target, 0.02, 0.04, 0.0);
    PrintGripperState("step", *gripper);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  // ── 第4组：阶梯渐闭 ──
  ARM_LOG_INFO("--- Round 4: step close 1.0 -> 0.6 -> 0.3 -> 0.0 ---");
  for (double target : {0.6, 0.3, 0.0}) {
    ARM_LOG_INFO("  -> target {:.1f}", target);
    gripper->SetPosition(target, 0.02, 0.04, 0.0);
    PrintGripperState("step", *gripper);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  ARM_LOG_INFO("=== Gripper Demo End ===");
  return 0;
}
