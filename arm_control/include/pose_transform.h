#pragma once

#include <Eigen/Dense>

#include "arm_type.h"

void Rotation2Euler(const double* R, double* euler);

void Euler2Rotation(const double* euler, double* R);

Eigen::Matrix4d PoseToTransform(const ArmMotion::Pose6Dof& pose);

ArmMotion::Pose6Dof TransformToPose(const Eigen::Matrix4d& T);
