#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

#include "arm_logger.h"
#include "robo_manager.h"

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
              << s.position.q[i] << std::setw(12) << s.velocity.q[i]
              << std::setw(12) << s.effort.q[i] << "\n";
  }
  std::cout << "\n";
}

namespace {
volatile std::sig_atomic_t g_running = 1;

void SignalHandler(int /*signum*/) { g_running = 0; }

}  // namespace

int main() {
  RoboManager mgr("geniarm_sim_config.yaml");
  auto arm = mgr.GetArmInstance();

  constexpr int kIntervalMs = 100;
  std::signal(SIGINT, SignalHandler);

  ARM_LOG_INFO("Read demo started, press Ctrl+C to stop.");

  int iter = 0;
  while (g_running) {
    ArmMotion::JointState_t state{};
    int ret = arm->GetCurrentJoint(state.position);

    if (ret == 0) {
      PrintJointState(("Read #" + std::to_string(++iter)).c_str(), state);
    } else {
      ARM_LOG_WARN("GetCurrentJoint failed");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(kIntervalMs));
  }

  ARM_LOG_INFO("Read demo stopped.");
  return 0;
}