#pragma once

#include "arm_type.h"

class JointPathBase
{
public:
    JointPathBase() = default;
    virtual ~JointPathBase() = default;

    virtual void EvaluatePos(double s, ArmMotion::Joint6Dof& point) const = 0;
    virtual void EvaluateTangent(ArmMotion::Joint6Dof& tangent) const = 0;
};