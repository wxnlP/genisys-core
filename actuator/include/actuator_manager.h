#pragma once

#include <string>
#include <vector>
#include <memory>
#include "arm_ctrl_base.h"
#include "gripper_ctrl_base.h"
#include "canfd_bus.h"
#include "dm_motor_driver.h"

class ActuatorManager {
 public:
  ActuatorManager(std::string file);

  void Shutdown();

  std::shared_ptr<ArmCtrlBase> GetArmActuator() { return arm_ctrl_; }
  std::shared_ptr<GripperCtrlBase> GetGripperActuator() { return gripper_ctrl_; }
  void GetJointNames(std::vector<std::string>& joint_names) { joint_names = joint_names_; }

 private:
  std::shared_ptr<ArmCtrlBase> arm_ctrl_;
  std::shared_ptr<GripperCtrlBase> gripper_ctrl_;
  std::shared_ptr<SocketCANBus> can_bus_;
  std::vector<DmMotorDriver> arm_motors_;
  std::shared_ptr<DmMotorDriver> gripper_motor_;
  std::vector<std::string> joint_names_;
};
