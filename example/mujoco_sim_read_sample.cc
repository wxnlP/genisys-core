#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

#include "mujoco_sim_ctrl.h"

static void PrintJointState(const char* label,
                            const ArmMotion::JointState_t& s) {
  const char* kJointNames[6] = {"shoulder", "upper_arm", "elbow",
                                "wrist1",   "wrist2",    "wrist3"};

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

namespace {
volatile std::sig_atomic_t g_running = 1;

void SignalHandler(int /*signum*/) { g_running = 0; }

}  // namespace

int main() {
  MujocoSimCtrl ctrl;
  if (!ctrl.Init()) {
    std::cerr << "[ERROR] MujocoSimCtrl::Init() failed\n";
    return 1;
  }

  constexpr int kIntervalMs = 100;
  std::signal(SIGINT, SignalHandler);

  std::cout << "[INFO] Read demo started, press Ctrl+C to stop.\n\n";

  int iter = 0;
  while (g_running) {
    ArmMotion::JointState_t state{};
    bool ok = ctrl.GetJointState(state);

    if (ok) {
      PrintJointState(("Read #" + std::to_string(++iter)).c_str(), state);
    } else {
      std::cerr << "[WARN] GetJointState failed\n\n";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kIntervalMs));
  }

  std::cout << "\n[INFO] Read demo stopped.\n";
  return 0;
}
