#pragma once

#include "joint_path_base.h"

class JointPath : public JointPathBase
{
public:
    JointPath(const ArmMotion::Joint6Dof_t& joint_start, const ArmMotion::Joint6Dof_t& joint_target)
        : joint_start_(joint_start), joint_target_(joint_target) {}

    ~JointPath() override = default;

    void EvaluatePos(double s, ArmMotion::Joint6Dof_t& point) const override;
    void EvaluateTangent(ArmMotion::Joint6Dof_t& tangent) const override;
private:
    ArmMotion::Joint6Dof_t joint_start_;
    ArmMotion::Joint6Dof_t joint_target_;
};