#pragma once

#include <memory>

#include "cartesian_path_base.h"

class CartesianLinearPath : public CartesianPathBase {
public:
    CartesianLinearPath(const ArmMotion::Pose6Dof& pose_start, const ArmMotion::Pose6Dof& pose_target);
    ~CartesianLinearPath() override;

    void EvaluatePos(double s, ArmMotion::Pose6Dof& point) const override;
    void EvaluateTangent(ArmMotion::Vector3d& pos, ArmMotion::Vector3d& omega) const override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
