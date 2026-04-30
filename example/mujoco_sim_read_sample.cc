#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

#include "arm_logger.h"
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
    ARM_LOG_ERROR("MujocoSimCtrl::Init() failed");
    return 1;
  }

  constexpr int kIntervalMs = 100;
  std::signal(SIGINT, SignalHandler);

  ARM_LOG_INFO("Read demo started, press Ctrl+C to stop.");

  int iter = 0;
  while (g_running) {
    ArmMotion::JointState_t state{};
    bool ok = ctrl.GetJointState(state);

    if (ok) {
      PrintJointState(("Read #" + std::to_string(++iter)).c_str(), state);
    } else {
      ARM_LOG_WARN("GetJointState failed");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kIntervalMs));
  }

  ARM_LOG_INFO("Read demo stopped.");
  return 0;
}
