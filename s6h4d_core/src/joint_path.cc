#include "joint_path.h"

void JointPath::EvaluatePos(double s, ArmMotion::Joint6Dof_t& point) const
{
    for (int i = 0; i < 6; ++i) {
        point.theta[i] = joint_start_.theta[i] + s * (joint_target_.theta[i] - joint_start_.theta[i]);
    }
}

void JointPath::EvaluateTangent(ArmMotion::Joint6Dof_t& tangent) const
{
    for (int i = 0; i < 6; ++i) {
        tangent.theta[i] = joint_target_.theta[i] - joint_start_.theta[i];
    }
}

