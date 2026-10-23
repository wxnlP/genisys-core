#include "pose_transform.h"

#include <cmath>

void Rotation2Euler(const double* R, double* euler) {
  Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>> m(R);

  double roll, pitch, yaw;

  double s_beta = -m(2, 0);
  double c_beta = sqrt(m(0, 0) * m(0, 0) + m(1, 0) * m(1, 0));

  if (c_beta > 0.000001) {
    roll = std::atan2(m(2, 1), m(2, 2));
    pitch = std::atan2(s_beta, c_beta);
    yaw = std::atan2(m(1, 0), m(0, 0));
  } else {
    yaw = 0.0;
    pitch = (s_beta > 0) ? (double)M_PI_2 : -(double)M_PI_2;
    roll = std::atan2(-m(0, 1), m(1, 1));
  }

  euler[0] = roll;
  euler[1] = pitch;
  euler[2] = yaw;
}

void Euler2Rotation(const double* euler, double* R) {
  Eigen::AngleAxisd roll(euler[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(euler[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(euler[2], Eigen::Vector3d::UnitZ());

  Eigen::Matrix3d m = (yaw * pitch * roll).toRotationMatrix();

  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::ColMajor>>(R, 3, 3) = m;
}

Eigen::Matrix4d PoseToTransform(const ArmMotion::Pose6Dof& pose) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  Eigen::Quaterniond q(pose.quat.w, pose.quat.x, pose.quat.y, pose.quat.z);
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }

  T.block<3, 3>(0, 0) = q.toRotationMatrix();
  T(0, 3) = pose.x;
  T(1, 3) = pose.y;
  T(2, 3) = pose.z;
  return T;
}

ArmMotion::Pose6Dof TransformToPose(const Eigen::Matrix4d& T) {
  ArmMotion::Pose6Dof pose;
  Eigen::Quaterniond q(T.block<3, 3>(0, 0));
  q.normalize();

  pose.x = T(0, 3);
  pose.y = T(1, 3);
  pose.z = T(2, 3);
  pose.quat = ArmMotion::Quaternion(q.w(), q.x(), q.y(), q.z());

  return pose;
}
