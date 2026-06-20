#pragma once

#include <cstddef>

namespace ArmMotion {

struct MDHParams {
  double alpha[6];
  double a[6];
  double d[6];
  double theta[6];
};

struct Quaternion {
  Quaternion() = default;
  Quaternion(double W, double X, double Y, double Z)
      : x(X), y(Y), z(Z), w(W) {}

  double x{0.0}, y{0.0}, z{0.0}, w{1.0};
};

struct Vector3d {
  Vector3d() = default;
  Vector3d(double X, double Y, double Z) : x(X), y(Y), z(Z) {}

  double x{0.0}, y{0.0}, z{0.0};
};

struct Joint6Dof {
  Joint6Dof() = default;

  Joint6Dof(double a1, double a2, double a3, double a4, double a5, double a6)
      : q{a1, a2, a3, a4, a5, a6} {}

  double& operator[](size_t i) { return q[i]; }
  const double& operator[](size_t i) const { return q[i]; }

  double q[6]{};
};

struct Pose6Dof {
  Pose6Dof() = default;

  Pose6Dof(double X, double Y, double Z, double qw, double qx, double qy,
             double qz)
      : x(X), y(Y), z(Z), quat{qw, qx, qy, qz}, vaildR(false) {}

  double x{}, y{}, z{};

  Quaternion quat{};
  double R[9]{};  // 列优先

  // 记录R变量的旋转矩阵是否有效
  bool vaildR{};
};

struct IKSolves {
  Joint6Dof solves[8];
  char solveFlag[8][3];
};

struct CartesianTwist {
  CartesianTwist() = default;
  CartesianTwist(double vx, double vy, double vz, double wx, double wy,
                   double wz)
      : linear{vx, vy, vz}, angular{wx, wy, wz} {}
  Vector3d linear{};
  Vector3d angular{};
};

struct JointTrajectoryPoint {
  double t{0.0};
  Joint6Dof position{};
  Joint6Dof velocity{};
  Joint6Dof acceleration{};
};

struct CartesianTrajectoryPoint {
  double t{0.0};
  Pose6Dof pose{};
  CartesianTwist velocity{};
  CartesianTwist acceleration{};
};

struct JointCommand {
  Joint6Dof position{};
  Joint6Dof velocity{};
  Joint6Dof effort{};
  Joint6Dof stiffness{};
  Joint6Dof damping{};
};

struct JointState {
  Joint6Dof position{};
  Joint6Dof velocity{};
  Joint6Dof effort{};
};
}  // namespace ArmMotion
