#pragma once

#include <memory>

#include "arm_type.h"

class KinematicSolver {
 public:
  enum class IKMode {
    SphericalWrist,
    ParallelWrist
  };
  explicit KinematicSolver(const ArmMotion::MDHParams& mdh_params);
  ~KinematicSolver();

  void SetIKMode(IKMode mode);

  bool FK(const ArmMotion::Joint6Dof& inputJoint,
          ArmMotion::Pose6Dof& outputPose);
  bool IK(const ArmMotion::Pose6Dof& inputPose, const ArmMotion::Joint6Dof& lastJoint,
          ArmMotion::IKSolves& outputSolves);

 private:
  bool IK_SphericalWrist(const ArmMotion::Pose6Dof& inputPose,
                         const ArmMotion::Joint6Dof& lastJoint,
                         ArmMotion::IKSolves& outputSolves);
  bool IK_ParallelWrist(const ArmMotion::Pose6Dof& inputPose,
                        const ArmMotion::Joint6Dof& lastJoint,
                        ArmMotion::IKSolves& outputSolves);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
  // 记录机械角度偏移
  double jointOffset_[6];
  IKMode ikMode_{IKMode::SphericalWrist};
};
