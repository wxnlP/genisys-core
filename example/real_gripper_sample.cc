#include <chrono>
#include <thread>
#include "arm_logger.h"
#include "canfd_bus.h"
#include "geniarm_gripper.h"

int main() {
  SocketCANBus can_bus("can0", true);
  int sock_fd = can_bus.Init();
  uint32_t ids[7] = {0x11, 0x17};
  can_bus.SetReceiverId(ids, 2);
  ARM_LOG_INFO("CAN BUS Init: {}", sock_fd);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::jthread sample_thread([&can_bus]() {
    ARM_LOG_INFO("sample_thread");
    while (true) {
      can_bus.Sample();
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  DmMotorDriver dm4310(can_bus, 0x07);

  GeniArmGripper gripper(dm4310, 12.0, 0.8);
  gripper.Enable();

  std::jthread receive_thread([&dm4310]() {
    DmMotorDriver::MotorState state{};
    while (true) {
      dm4310.RequestState();
      int ret = dm4310.GetLatestState(state);
      if (ret != -1) {
        ARM_LOG_INFO("==================Motor State=====================");
        ARM_LOG_INFO("id: {} pos: {} vel: {} torque: {}",
                     state.id, state.pos, state.vel, state.torque);
        ARM_LOG_INFO("==================================================");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  DmMotorDriver::MotorState state;

  bool ret = gripper.SetPosition(1, 0.5, 1, 0);
  ARM_LOG_INFO("SetPosition: {}", ret);
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  gripper.SetPosition(0.0, 0.5, 1, 0);
  gripper.Disable();
  return 0;
}