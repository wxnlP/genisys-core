#pragma once

#include "cartesian_path_base.h"
#include <Eigen/Dense>

class CartesianLinearPath : public CartesianPathBase {
public:
    CartesianLinearPath(const ArmMotion::Pose6Dof_t& pose_start, const ArmMotion::Pose6Dof_t& pose_target);
    ~CartesianLinearPath() override = default;

    void EvaluatePos(double s, ArmMotion::Pose6Dof_t& point) const override;
    void EvaluateTangent(ArmMotion::Vector3d& pos, ArmMotion::Vector3d& omega) const override;

private:
    // 位置
    Eigen::Vector3d p_start_, p_target_;
    // 姿态
    Eigen::Quaterniond q_start_, q_target_;
};
