#pragma once

#include <memory>
#include <string>
#include "arm_controller.h"
#include "gripper_controller.h"

class ArmManager {
 public:
  ArmManager(const std::string& yaml_file);

  std::shared_ptr<ArmController> GetArmController() { return arm_controller_; }
  std::shared_ptr<GripperController> GetGripperController() { return gripper_controller_; }

 private:
  std::shared_ptr<ArmController> arm_controller_;
  std::shared_ptr<GripperController> gripper_controller_;
};
