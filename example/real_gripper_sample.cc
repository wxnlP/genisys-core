#include <chrono>
#include <thread>
#include "arm_logger.h"
#include "canfd_bus.h"
#include "geniarm_gripper.h"
#include "gripper_controller.h"

int main() {
  /* Socket CAN Init */
  SocketCANBus can_bus("can0", true);
  int sock_fd = can_bus.Init();
  uint32_t ids[7] = {0x11, 0x17};
  can_bus.SetReceiverId(ids, 2);
  ARM_LOG_INFO("CAN BUS Init: {}", sock_fd);

  /* Motor Driver Init */
  DmMotorDriver dm4340p(can_bus, 0x07, DmMotorDriver::MotorModel::DM4340P);
  DmMotorDriver dm4310(can_bus, 0x07, DmMotorDriver::MotorModel::DM4310);


  /* GeniArm Gripper Init */
  GeniArmGripper gripper(dm4310, 12.0, 1);
  gripper.SetMaxRange(-2.2);
  gripper.Init();
  gripper.Enable();

  GripperController gripper_ctrl(gripper);

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

  /* 位置速度控制 */
  // bool ret = gripper.SetPosition(1, 0.5, 1, 0);
  // ARM_LOG_INFO("SetPosition: {}", ret);
  // std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  // gripper.SetPosition(0.0, 0.5, 1, 0);
  // gripper.Disable();

  /* 力控 */
  gripper_ctrl.SetPosition(0.4, 0.5, 1, 0);
 
  ARM_LOG_INFO("=== Test 1: -3Nm torque ===");
  gripper_ctrl.SetTorque(-3.0, 1.0);   // 1Nm，2Nm/s 斜坡
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  ARM_LOG_INFO("回归零点......");
  gripper_ctrl.SetPosition(0.0, 0.5, 1, 0);
  gripper.Disable();
  return 0;
}