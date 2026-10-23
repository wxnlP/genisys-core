#include "tool_frame.h"

#include "pose_transform.h"

void ToolFrame::Set(const ArmMotion::Pose6Dof& tool_pose) {
  T_6tool_ = PoseToTransform(tool_pose);
  T_tool6_ = T_6tool_.inverse();
  configured_ = true;
}

void ToolFrame::Clear() {
  T_6tool_.setIdentity();
  T_tool6_.setIdentity();
  configured_ = false;
}

ArmMotion::Pose6Dof ToolFrame::Compose(const ArmMotion::Pose6Dof& flange_pose) const {
  return TransformToPose(PoseToTransform(flange_pose) * T_6tool_);
}

ArmMotion::Pose6Dof ToolFrame::Remove(const ArmMotion::Pose6Dof& tcp_pose) const {
  return TransformToPose(PoseToTransform(tcp_pose) * T_tool6_);
}

ArmMotion::Vector3d ToolFrame::LocalPoint() const {
  return {T_6tool_(0, 3), T_6tool_(1, 3), T_6tool_(2, 3)};
}
