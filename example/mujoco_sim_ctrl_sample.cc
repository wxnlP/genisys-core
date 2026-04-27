#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

#include "mujoco_sim_ctrl.h"

// ──────────────────────────────────────────────
// 打印 JointState_t 的各字段
// ──────────────────────────────────────────────
static void PrintJointState(const char* label,
                             const ArmMotion::JointState_t& s) {
  const char* kJointNames[6] = {"shoulder", "upper_arm", "elbow",
                                 "wrist1",   "wrist2",    "wrist3"};

  std::cout << "=== " << label << " ===\n";
  std::cout << std::fixed << std::setprecision(4);
  std::cout << std::left << std::setw(12) << "joint"
            << std::setw(12) << "pos(rad)"
            << std::setw(12) << "vel(rad/s)"
            << std::setw(12) << "effort(Nm)" << "\n";
  std::cout << std::string(48, '-') << "\n";
  for (int i = 0; i < 6; ++i) {
    std::cout << std::left << std::setw(12) << kJointNames[i]
              << std::setw(12) << s.position.theta[i]
              << std::setw(12) << s.velocity.theta[i]
              << std::setw(12) << s.effort.theta[i] << "\n";
  }
  std::cout << "\n";
}

int main() {
  MujocoSimCtrl ctrl;

  // ── 初始化 ──────────────────────────────────
  if (!ctrl.Init()) {
    std::cerr << "[ERROR] MujocoSimCtrl::Init() failed\n";
    return 1;
  }
  std::cout << "[INFO] MujocoSimCtrl initialized\n\n";

  // ── 测试 SetJointState ──────────────────────
  // 构造一个测试指令：各关节角度逐步递增
  ArmMotion::JointState_t cmd{};
  cmd.position = {0.0, 0.0, -1.88, 0.0, 0.0, 0.0};  // 与 send_joint_cmd.sh 默认值一致
  cmd.velocity = {};
  cmd.effort   = {};

  PrintJointState("SetJointState command", cmd);

  bool ok_set = ctrl.SetJointState(cmd);
  if (ok_set) {
    std::cout << "[INFO] SetJointState: OK (HTTP 200)\n\n";
  } else {
    std::cerr << "[WARN] SetJointState: FAILED (check if simulator is running "
                 "at http://127.0.0.1:50080)\n\n";
  }

  // 等待仿真响应稳定
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // ── 测试 GetJointState ──────────────────────
  ArmMotion::JointState_t state{};
  bool ok_get = ctrl.GetJointState(state);
  if (ok_get) {
    PrintJointState("GetJointState response", state);
  } else {
    std::cerr << "[WARN] GetJointState: FAILED (check if /mujoco/joint_state "
                 "channel is publishing)\n\n";
  }

  // ── 简单往返测试：发送 → 读取 × 3 次 ──────
  std::cout << "[INFO] Round-trip test (3 iterations)...\n";
  for (int iter = 0; iter < 3; ++iter) {
    // 每次改变 elbow 角度（0.5rad 步进）
    cmd.position.theta[2] = -1.88 + iter * 0.5;
    PrintJointState(("Send #" + std::to_string(iter + 1)).c_str(), cmd);

    ok_set = ctrl.SetJointState(cmd);
    std::cout << "  SetJointState: " << (ok_set ? "OK" : "FAILED") << "\n";

    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    ok_get = ctrl.GetJointState(state);
    if (ok_get) {
      PrintJointState(("Recv #" + std::to_string(iter + 1)).c_str(), state);
    } else {
      std::cout << "  GetJointState: FAILED\n\n";
    }
  }

  return 0;
}
