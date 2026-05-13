#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

#include "arm_logger.h"
#include "sim_ctrl.h"

// ──────────────────────────────────────────────
// 打印 JointState_t 的各字段
// ──────────────────────────────────────────────
static void PrintJointState(const char* label,
                            const ArmMotion::JointState_t& s) {
  const char* kJointNames[6] = {"shoulder", "upper_arm", "elbow",
                                "wrist1", "wrist2", "wrist3"};

  std::cout << "=== " << label << " ===\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << std::left << std::setw(12) << "joint" << std::setw(12)
            << "pos(rad)" << std::setw(12) << "vel(rad/s)" << std::setw(12)
            << "effort(Nm)" << "\n";
  std::cout << std::string(48, '-') << "\n";
  for (int i = 0; i < 6; ++i) {
    std::cout << std::left << std::setw(12) << kJointNames[i] << std::setw(12)
              << s.position.theta[i] << std::setw(12) << s.velocity.theta[i]
              << std::setw(12) << s.effort.theta[i] << "\n";
  }
  std::cout << "\n";
}

int main() {
  SimCtrl ctrl;
  if (!ctrl.Init()) {
    ARM_LOG_ERROR("SimCtrl::Init() failed");
    return 1;
  }

  constexpr int kIntervalMs = 200;
  constexpr int kPoints = 10;
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDegToRad = kPi / 180.0;

  // 小幅动作基准点。
  const double base[6] = {0.0, 0.1, -1.4, 0.1, -0.1, 0.0};
  const double amp[6] = {8.0 * kDegToRad, 10.0 * kDegToRad, 12.0 * kDegToRad,
                         9.0 * kDegToRad, 7.0 * kDegToRad, 6.0 * kDegToRad};
  const double phase[6] = {0.0, 0.5, 1.0, 1.5, 2.0, 2.5};

  ARM_LOG_INFO("Send demo: publish {} points", kPoints);

  for (int i = 0; i < kPoints; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(kPoints - 1);

    ArmMotion::JointState_t cmd{};
    for (int j = 0; j < 6; ++j) {
      cmd.position.theta[j] =
          base[j] + amp[j] * std::sin(2.0 * kPi * t + phase[j]);
      cmd.velocity.theta[j] = 0.0;
      cmd.effort.theta[j] = 0.0;
    }

    PrintJointState(("Send #" + std::to_string(i + 1)).c_str(), cmd);

    const bool ok = ctrl.SetJointState(cmd);
    ARM_LOG_INFO("SetJointState: {}", ok ? "OK" : "FAILED");

    std::this_thread::sleep_for(std::chrono::milliseconds(kIntervalMs));
  }

  return 0;
}