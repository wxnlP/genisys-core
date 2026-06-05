#include <chrono>
#include <thread>
#include "arm_logger.h"
#include "canfd_bus.h"
#include "geniarm_gripper.h"

int main() {
  /* Socket CAN Init */
  SocketCANBus can_bus("can0", true);
  int sock_fd = can_bus.Init();
  uint32_t ids[7] = {0x11, 0x17};
  can_bus.SetReceiverId(ids, 2);
  ARM_LOG_INFO("CAN BUS Init: {}", sock_fd);

  /* Motor Driver Init */
  DmMotorDriver dm4310(can_bus, 0x07);

  /* GeniArm Gripper Init */
  GeniArmGripper gripper(dm4310, 12.0, 0.8);
  gripper.SetMaxRange(1.9);
  gripper.Init();
  gripper.Enable();

  std::jthread receive_thread([&gripper]() {
    double position, velocity, effort;
    while (true) {
      int ret = gripper.ReceiveState(position, velocity, effort);
      if (ret != -1) {
        ARM_LOG_INFO("==================Motor State=====================");
        ARM_LOG_INFO("pos: {} vel: {} torque: {}", position, velocity, effort);
        ARM_LOG_INFO("==================================================");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  bool ret = gripper.SetPosition(1, 0.5, 1, 0);
  ARM_LOG_INFO("SetPosition: {}", ret);
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  gripper.SetPosition(0.0, 0.5, 1, 0);
  gripper.Disable();
  return 0;
}