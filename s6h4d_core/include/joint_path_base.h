#pragma once

#include "arm_type.h"

class JointPathBase
{
public:
    JointPathBase() = default;
    virtual ~JointPathBase() = default;

    virtual void EvaluatePos(double s, ArmMotion::Joint6Dof_t& point) const = 0;
    virtual void EvaluateTangent(double s, ArmMotion::Joint6Dof_t& tangent) const = 0;
};