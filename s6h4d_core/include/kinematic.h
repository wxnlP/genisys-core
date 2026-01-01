#pragma once

#include <memory>
#include "arm_type.h"

class KinematicSolver
{
public:
    KinematicSolver(double L_BS, double D_BS, double L_AM, double L_FA, double D_EW, double L_WT);
    ~KinematicSolver();
    KinematicSolver(const KinematicSolver&) = delete;
    KinematicSolver& operator=(const KinematicSolver&) = delete;

    bool FK(const ArmMotion::Joint6Dof_t &inputJoint, ArmMotion::Pose6Dof_t &outputPose);
    bool IK(const ArmMotion::Pose6Dof_t &inputPose, const ArmMotion::Joint6Dof_t &lastJoint, ArmMotion::IKSolves_t &outputSolves);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    // 记录机械角度偏移
    double jointOffset_[6];
};
