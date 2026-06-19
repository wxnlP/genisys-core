#pragma once

#include <memory>
#include <string>

#include "arm_ctrl_base.h"
#include "arm_instance.h"
#include "gripper_controller.h"
#include "gripper_ctrl_base.h"

class RoboManager {
 public:
  explicit RoboManager(const std::string& yaml_file);
  ~RoboManager();

  // 禁止拷贝
  RoboManager(const RoboManager&) = delete;
  RoboManager& operator=(const RoboManager&) = delete;
  // 允许移动
  RoboManager(RoboManager&&) noexcept;
  RoboManager& operator=(RoboManager&&) noexcept;

  std::shared_ptr<ArmInstance> GetArmInstance() { return arm_instance_; }
  std::shared_ptr<GripperController> GetGripperController() { return gripper_controller_; }

 private:
  std::shared_ptr<ArmInstance> arm_instance_;
  std::shared_ptr<ArmCtrlBase> arm_ctrl_;
  std::shared_ptr<GripperCtrlBase> gripper_;
  std::shared_ptr<GripperController> gripper_controller_;

  // Pimpl：隔离真机硬件依赖（dm_motor / canfd_bus）
  class Impl;
  std::unique_ptr<Impl> impl_;
};