#pragma once

namespace ArmMotion {

struct MDHConfig_t {
  double L_BASE;
  double D_BASE;
  double L_UPPERARM;
  double D_ELBOW;
  double L_FOREARM;
  double L_WRIST;
};

struct MDHParams {
  double alpha[6];
  double a[6];
  double d[6];
  double theta[6];
};

struct Quaternion_t {
  Quaternion_t() = default;
  Quaternion_t(double W, double X, double Y, double Z)
      : x(X), y(Y), z(Z), w(W) {}

  double x{0.0}, y{0.0}, z{0.0}, w{1.0};
};

struct Vector3d {
  Vector3d() = default;
  Vector3d(double X, double Y, double Z) : x(X), y(Y), z(Z) {}

  double x{0.0}, y{0.0}, z{0.0};
};

struct Joint6Dof_t {
  Joint6Dof_t() = default;

  Joint6Dof_t(double a1, double a2, double a3, double a4, double a5, double a6)
      : theta{a1, a2, a3, a4, a5, a6} {}

  double theta[6]{};
};

struct Pose6Dof_t {
  Pose6Dof_t() = default;

  Pose6Dof_t(double X, double Y, double Z, double qw, double qx, double qy,
             double qz)
      : x(X), y(Y), z(Z), quat{qw, qx, qy, qz}, vaildR(false) {}

  double x{}, y{}, z{};

  Quaternion_t quat{};
  double R[9]{};  // 列优先

  // 记录R变量的旋转矩阵是否有效
  bool vaildR{};
};

struct IKSolves_t {
  Joint6Dof_t solves[8];
  char solveFlag[8][3];
};

struct CartesianTwist_t {
  CartesianTwist_t() = default;
  CartesianTwist_t(double vx, double vy, double vz, double wx, double wy,
                   double wz)
      : linear{vx, vy, vz}, angular{wx, wy, wz} {}
  Vector3d linear{};
  Vector3d angular{};
};

struct JointTrajectoryPoint_t {
  double t{0.0};
  double s{0.0};
  double sd{0.0};
  double sdd{0.0};
  Joint6Dof_t position{};
  Joint6Dof_t velocity{};
  Joint6Dof_t acceleration{};
};

struct CartesianTrajectoryPoint_t {
  double t{0.0};
  double s{0.0};
  double sd{0.0};
  double sdd{0.0};
  Pose6Dof_t pose{};
  CartesianTwist_t velocity{};
  CartesianTwist_t acceleration{};
};

struct JointState_t {
  Joint6Dof_t position{};
  Joint6Dof_t velocity{};
  Joint6Dof_t accel{};
  Joint6Dof_t effort{};
};
}  // namespace ArmMotion
