#pragma once

#include "arm_type.h"

class CartesianPathBase
{
public:
    CartesianPathBase() = default;
    virtual ~CartesianPathBase() = default;

    virtual void EvaluatePos(double s, ArmMotion::Pose6Dof& point) const = 0;
    virtual void EvaluateTangent(ArmMotion::Vector3d& pos, ArmMotion::Vector3d& omega) const = 0;
};