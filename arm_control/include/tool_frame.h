#pragma once

#include <Eigen/Dense>

#include "arm_type.h"

// 法兰坐标系与工具坐标系的固定变换
class ToolFrame {
 public:
  ToolFrame() = default;

  void Set(const ArmMotion::Pose6Dof& tool_pose);
  void Clear();
  bool configured() const { return configured_; }

  // 法兰位姿 → TCP 位姿
  ArmMotion::Pose6Dof Compose(const ArmMotion::Pose6Dof& flange_pose) const;
  // TCP 位姿 → 法兰位姿
  ArmMotion::Pose6Dof Remove(const ArmMotion::Pose6Dof& tcp_pose) const;
  // 工具点在法兰坐标系下的位置
  ArmMotion::Vector3d LocalPoint() const;

 private:
  Eigen::Matrix4d T_6tool_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_tool6_{Eigen::Matrix4d::Identity()};
  bool configured_{false};
};
