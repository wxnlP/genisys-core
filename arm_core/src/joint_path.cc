#include "joint_path.h"

void JointPath::EvaluatePos(double s, ArmMotion::Joint6Dof_t& point) const
{
    for (int i = 0; i < 6; ++i) {
        point.q[i] = joint_start_.q[i] + s * (joint_target_.q[i] - joint_start_.q[i]);
    }
}

void JointPath::EvaluateTangent(ArmMotion::Joint6Dof_t& tangent) const
{
    for (int i = 0; i < 6; ++i) {
        tangent.q[i] = joint_target_.q[i] - joint_start_.q[i];
    }
}

