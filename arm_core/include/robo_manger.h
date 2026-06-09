#pragma once

#include "arm_instance.h"
#include "geniarm_ctrl.h"
#include "geniarm_gripper.h"
#include "s6h4d_ctrl.h"
#include "dm_motor_driver.h"
#include "canfd_bus.h"

class RoboManager {
 public:
  RoboManager(const std::string& yaml_file);
  ~RoboManager();

  std::shared_ptr<ArmInstance> GetArmInstance() { return arm_instance_; }
  std::shared_ptr<GeniArmGripper> GetGripper() { return gripper_; }

 private:
  std::shared_ptr<ArmInstance> arm_instance_;
  std::shared_ptr<GeniArmGripper> gripper_;

  std::shared_ptr<SocketCANBus> can_bus_;
  std::vector<DmMotorDriver> arm_motors_;
  std::shared_ptr<DmMotorDriver> gripper_motor_;
  std::shared_ptr<GeniArmCtrl> arm_ctrl_;
};