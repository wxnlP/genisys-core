#include "kinematic.h"

#include <Eigen/Dense>
#include <cmath>
#include <cstring>
#include <iostream>

static void Rotation2Euler(const double* R, double* euler) {
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

  euler[0] = roll;   // roll
  euler[1] = pitch;  // pitch
  euler[2] = yaw;    // yaw
}

static void Euler2Rotation(const double* euler, double* R) {
  Eigen::AngleAxisd roll(euler[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch(euler[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw(euler[2], Eigen::Vector3d::UnitZ());

  Eigen::Matrix3d m = (yaw * pitch * roll).toRotationMatrix();

  // 修复：在构造Eigen::Map对象时直接传入指针R，然后赋值
  Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::ColMajor>>(R, 3, 3) = m;
}

static Eigen::Matrix4d PoseToTransform(const ArmMotion::Pose6Dof_t& pose) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R;

  Eigen::Quaterniond q(pose.quat.w, pose.quat.x, pose.quat.y, pose.quat.z);
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  R = q.toRotationMatrix();
  

  T.block<3, 3>(0, 0) = R;
  T(0, 3) = pose.x;
  T(1, 3) = pose.y;
  T(2, 3) = pose.z;
  return T;
}

static ArmMotion::Pose6Dof_t TransformToPose(const Eigen::Matrix4d& T) {
  ArmMotion::Pose6Dof_t pose;
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Quaterniond q(R);
  q.normalize();

  pose.x = T(0, 3);
  pose.y = T(1, 3);
  pose.z = T(2, 3);
  pose.quat = ArmMotion::Quaternion_t(q.w(), q.x(), q.y(), q.z());

  return pose;
}


class KinematicSolver::Impl {
 public:
  // MDH params
  ArmMotion::MDHParams mdhParams_{};

  // Eigen::Matrix<double, 6, 4> MDH_Matrix_;

  // 旧版本变量
  // Eigen::Vector3d L0_Base_;
  // Eigen::Vector3d L2_Arm_;
  // Eigen::Vector3d L3_Elbow_;
  // Eigen::Vector3d L6_Wrist_;

  // 新版本变量
  Eigen::Vector3d P_02;
  Eigen::Vector3d P_23;
  Eigen::Vector3d P_34;
  Eigen::Vector3d P_65;
  Eigen::Matrix4d T_6tool{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_tool6{Eigen::Matrix4d::Identity()};
  bool toolPoseEnabled{false};

  explicit Impl(const ArmMotion::MDHParams& mdh_params)
      : mdhParams_(mdh_params) {
    // 备份
    // MDH_Matrix_ <<
    //     mdh_params.alpha[0], mdh_params.a[0], mdh_params.d[0], mdh_params.theta[0],
    //     mdh_params.alpha[1], mdh_params.a[1], mdh_params.d[1], mdh_params.theta[1],
    //     mdh_params.alpha[2], mdh_params.a[2], mdh_params.d[2], mdh_params.theta[2],
    //     mdh_params.alpha[3], mdh_params.a[3], mdh_params.d[3], mdh_params.theta[3],
    //     mdh_params.alpha[4], mdh_params.a[4], mdh_params.d[4], mdh_params.theta[4],
    //     mdh_params.alpha[5], mdh_params.a[5], mdh_params.d[5], mdh_params.theta[5];

    P_02 = Eigen::Vector3d(mdhParams_.a[1], 0.0, mdhParams_.d[0]);
    P_23 = Eigen::Vector3d(mdhParams_.a[2], 0.0, 0.0);
    P_34 = Eigen::Vector3d(mdhParams_.a[3], mdhParams_.d[3], 0.0);
    P_65 = Eigen::Vector3d(0.0, 0.0, -mdhParams_.d[5]);
  }
};

KinematicSolver::KinematicSolver(const ArmMotion::MDHParams& mdh_params)
    : impl_(std::make_unique<Impl>(mdh_params)) {
  jointOffset_[0] = mdh_params.theta[0];
  jointOffset_[1] = mdh_params.theta[1];
  jointOffset_[2] = mdh_params.theta[2];
  jointOffset_[3] = mdh_params.theta[3];
  jointOffset_[4] = mdh_params.theta[4];
  jointOffset_[5] = mdh_params.theta[5];
}

bool KinematicSolver::FK(const ArmMotion::Joint6Dof_t& inputJoint,
                         ArmMotion::Pose6Dof_t& outputPose) {
  double theta_in[6];
  double theta[6];

  Eigen::Matrix4d T[6];
  Eigen::Matrix4d T_02, T_03, T_04, T_05, T_06;
  Eigen::Matrix3d R_06;

  for (int i = 0; i < 6; i++) {
    theta_in[i] = inputJoint.q[i];
  }

  // 处理齐次变换矩阵
  for (int i = 0; i < 6; i++) {
    theta[i] = theta_in[i] + jointOffset_[i];

    double st = std::sin(theta[i]);
    double ct = std::cos(theta[i]);
    double sa = std::sin(impl_->mdhParams_.alpha[i]);
    double ca = std::cos(impl_->mdhParams_.alpha[i]);
    double a = impl_->mdhParams_.a[i];
    double d = impl_->mdhParams_.d[i];

    T[i] << ct, -st, 0.0, a,
        st * ca, ct * ca, -sa, -d * sa,
        st * sa, ct * sa, ca, d * ca,
        0.0, 0.0, 0.0, 1.0;
  }

  T_02 = T[0] * T[1];
  T_03 = T_02 * T[2];
  T_04 = T_03 * T[3];
  T_05 = T_04 * T[4];
  T_06 = T_05 * T[5];

  // posture
  R_06 = T_06.block<3, 3>(0, 0);
  Eigen::Quaterniond P_06_quat(R_06);
  P_06_quat.normalize();

  Eigen::Matrix4d T_0tcp = T_06 * impl_->T_6tool;
  outputPose = TransformToPose(T_0tcp);

  return true;
}

bool KinematicSolver::IK_SphericalWrist(const ArmMotion::Pose6Dof_t& inputPose,
                                        const ArmMotion::Joint6Dof_t& lastJoint,
                                        ArmMotion::IKSolves_t& outputSolves) {
  double theta_1[2]{};
  double theta_23[2][2]{};
  double theta_456[2][3]{};

  Eigen::Vector3d P_06_pos;
  Eigen::Quaterniond P_06_quat;
  Eigen::Vector3d P_0w;
  Eigen::Vector3d L_2w;
  Eigen::Matrix3d R_10;
  Eigen::Matrix3d R_31;
  Eigen::Matrix3d R_30;
  Eigen::Matrix3d R_36;
  Eigen::Matrix3d R_06;

  Eigen::Vector3d L0_Wrist;

  P_06_pos[0] = inputPose.x;
  P_06_pos[1] = inputPose.y;
  P_06_pos[2] = inputPose.z;

  if (inputPose.vaildR) {
    memcpy(R_06.data(), inputPose.R, 9 * sizeof(double));
  } else {
    // deg to rad
    P_06_quat = Eigen::Quaterniond(inputPose.quat.w, inputPose.quat.x,
                                   inputPose.quat.y, inputPose.quat.z);
    // P_06_quat.x() = inputPose.qx;
    // P_06_quat.y() = inputPose.qy;
    // P_06_quat.z() = inputPose.qz;
    // P_06_quat.w() = inputPose.qw;

    // quaternion to rotation
    P_06_quat.normalize();
    R_06 = P_06_quat.toRotationMatrix();
    // Euler2Rotation(P_06_euler.data(), R_06.data());
  }

  for (int i = 0; i < 2; i++) {
    theta_1[i] = lastJoint.q[0] + jointOffset_[0];
    theta_23[i][0] = lastJoint.q[1] + jointOffset_[1];
    theta_23[i][1] = lastJoint.q[2] + jointOffset_[2];
    theta_456[i][0] = lastJoint.q[3] + jointOffset_[3];
    theta_456[i][1] = lastJoint.q[4] + jointOffset_[4];
    theta_456[i][2] = lastJoint.q[5] + jointOffset_[5];
  }

  L0_Wrist = R_06 * impl_->P_65;

  P_0w.head<3>() = P_06_pos + L0_Wrist;

  /* ---------------------------------J1--------------------------------------
   */
  if ((P_0w[0] * P_0w[0] + P_0w[1] * P_0w[1]) <= 0.000001) {
    theta_1[0] = lastJoint.q[0] + jointOffset_[0];
    theta_1[1] = theta_1[0];
    for (int i = 0; i < 4; i++) {
      outputSolves.solveFlag[0 + i][0] = -1;
      outputSolves.solveFlag[4 + i][0] = -1;
    }
  } else {
    theta_1[0] = std::atan2(P_0w[1], P_0w[0]);
    theta_1[1] = std::atan2(-P_0w[1], -P_0w[0]);
    for (int i = 0; i < 4; i++) {
      outputSolves.solveFlag[0 + i][0] = 1;
      outputSolves.solveFlag[4 + i][0] = 1;
    }
  }

  /* ---------------------------------J23-------------------------------------
   */
  for (int ind_arm = 0; ind_arm < 2; ind_arm++) {
    double ct1 = std::cos(theta_1[ind_arm]);
    double st1 = std::sin(theta_1[ind_arm]);

    R_10 << ct1, st1, 0.0, -st1, ct1, 0.0, 0.0, 0.0, 1.0;

    Eigen::Vector3d P_0w_r;

    // rotation
    P_0w_r = R_10 * P_0w;

    L_2w = P_0w_r - impl_->P_02;

    double c = sqrt(L_2w[0] * L_2w[0] + L_2w[2] * L_2w[2]);
    double b = sqrt(impl_->mdhParams_.a[3] * impl_->mdhParams_.a[3] +
                    impl_->mdhParams_.d[3] * impl_->mdhParams_.d[3]);
    double a = impl_->mdhParams_.a[2];

    if (fabs(a + b - c) <= 0.000001) {
      theta_23[0][0] = std::atan2(L_2w[2], L_2w[0]);
      theta_23[1][0] = theta_23[0][0];

      theta_23[0][1] = 0.0;
      theta_23[1][1] = 0.0;

      if (c > a + b) {
        for (int i = 0; i < 2; i++) {
          outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 0;
          outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 0;
        }
      } else {
        for (int i = 0; i < 2; i++) {
          outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 1;
          outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 1;
        }
      }
    } else if (fabs(c - fabs(a - b) <= 0.000001)) {
      theta_23[0][0] = std::atan2(L_2w[2], L_2w[0]);
      theta_23[1][0] = theta_23[0][0];

      if (ind_arm == 0) {
        theta_23[0][1] = (double)M_PI;
        theta_23[1][1] = -(double)M_PI;
      } else {
        theta_23[0][1] = -(double)M_PI;
        theta_23[1][1] = (double)M_PI;
      }

      if (c < fabs(a - b)) {
        for (int i = 0; i < 2; i++) {
          outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 0;
          outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 0;
        }
      } else {
        for (int i = 0; i < 2; i++) {
          outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 1;
          outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 1;
        }
      }
    } else {
      double beta2, beta3;
      double q3 = (a * a + b * b - c * c) / (2 * a * b);

      if (q3 >= 1.0) {
        beta3 = 0.0;
      } else if (q3 <= -1.0) {
        beta3 = (double)M_PI;
      } else {
        beta3 = std::acos(q3);
      }

      double q2 = (a * a + c * c - b * b) / (2 * a * c);
      if (q2 >= 1.0) {
        beta2 = 0.0;
      } else if (q2 <= -1.0) {
        beta2 = (double)M_PI;
      } else {
        beta2 = std::acos(q2);
      }

      double alpha2 = std::atan2(L_2w[2], L_2w[0]);
      double alpha3 =
          std::atan2(impl_->mdhParams_.d[3], impl_->mdhParams_.a[3]);

      if (ind_arm == 0) {
        theta_23[0][0] = alpha2 + beta2;
        theta_23[0][1] = -((double)M_PI - alpha3 - beta3);
        theta_23[1][0] = -(alpha2 + beta2);
        theta_23[1][1] = (double)M_PI - alpha3 - beta3;
      } else {
        theta_23[0][0] = -(alpha2 + beta2);
        theta_23[0][1] = (double)M_PI - alpha3 - beta3;
        theta_23[1][0] = alpha2 + beta2;
        theta_23[1][1] = -((double)M_PI - alpha3 - beta3);
      }
      for (int i = 0; i < 2; i++) {
        outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 1;
        outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 1;
      }
    }
    /* ---------------------------------J456------------------------------------- */
    for (int ind_elbow = 0; ind_elbow < 2; ind_elbow++) {
      double c2 = std::cos(theta_23[ind_elbow][0]);
      double s2 = std::sin(theta_23[ind_elbow][0]);
      double c3 = std::cos(theta_23[ind_elbow][1]);
      double s3 = std::sin(theta_23[ind_elbow][1]);

      R_31 << (c2 * c3 - s2 * s3), 0.0, -(c2 * s3 + s2 * c3),
          -(c2 * s3 + s2 * c3), 0.0, -(c2 * c3 - s2 * s3), 0.0, 1.0, 0.0;

      R_30 = R_31 * R_10;
      R_36 = R_30 * R_06;

      double cos5;
      if (R_36(1, 2) > 1.0 - 0.00001) {
        cos5 = 1.0;
        theta_456[0][1] = 0.0;
        theta_456[1][1] = 0.0;
      } else if (R_36(1, 2) < -1.0 + 0.00001) {
        cos5 = -1.0;
        if (ind_arm == 0) {
          theta_456[0][1] = (double)M_PI;
          theta_456[1][1] = -(double)M_PI;
        } else {
          theta_456[0][1] = -(double)M_PI;
          theta_456[1][1] = (double)M_PI;
        }
      } else {
        cos5 = R_36(1, 2);
        if (ind_arm == 0) {
          theta_456[0][1] = std::acos(cos5);
          theta_456[1][1] = -std::acos(cos5);
        } else {
          theta_456[0][1] = -std::acos(cos5);
          theta_456[1][1] = std::acos(cos5);
        }
      }
      if (cos5 == 1.0 || cos5 == -1.0) {
        if (ind_arm == 0) {
          theta_456[0][0] = lastJoint.q[3] + jointOffset_[3];
          double c4 = std::cos(lastJoint.q[3] + jointOffset_[3]);
          double s4 = std::sin(lastJoint.q[3] + jointOffset_[3]);
          theta_456[0][2] = std::atan2(-s4 * R_36(0, 0) - c4 * R_36(2, 0),
                                       -s4 * R_36(0, 1) - c4 * R_36(2, 1));

          theta_456[1][2] = lastJoint.q[5] + jointOffset_[5];
          double c6 = std::cos(lastJoint.q[5] + jointOffset_[5]);
          double s6 = std::sin(lastJoint.q[5] + jointOffset_[5]);
          theta_456[1][0] = std::atan2(-s6 * R_36(0, 0) - c6 * R_36(0, 1),
                                       -s6 * R_36(2, 0) - c6 * R_36(2, 1));
        } else {
          theta_456[0][2] = lastJoint.q[5] + jointOffset_[5];
          double c6 = std::cos(lastJoint.q[5] + jointOffset_[5]);
          double s6 = std::sin(lastJoint.q[5] + jointOffset_[5]);
          theta_456[0][0] = std::atan2(-s6 * R_36(0, 0) - c6 * R_36(0, 1),
                                       -s6 * R_36(2, 0) - c6 * R_36(2, 1));

          theta_456[1][0] = lastJoint.q[3] + jointOffset_[3];
          double c4 = std::cos(lastJoint.q[3] + jointOffset_[3]);
          double s4 = std::sin(lastJoint.q[3] + jointOffset_[3]);
          theta_456[1][2] = std::atan2(-s4 * R_36(0, 0) - c4 * R_36(2, 0),
                                       -s4 * R_36(0, 1) - c4 * R_36(2, 1));
        }

        outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = -1;
        outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = -1;
      } else {
        if (ind_arm == 0) {
          theta_456[0][0] = std::atan2(R_36(2, 2), -R_36(0, 2));
          theta_456[1][0] = std::atan2(-R_36(2, 2), R_36(0, 2));
          theta_456[0][2] = std::atan2(-R_36(1, 1), R_36(1, 0));
          theta_456[1][2] = std::atan2(R_36(1, 1), -R_36(1, 0));
        } else {
          theta_456[1][0] = std::atan2(R_36(2, 2), -R_36(0, 2));
          theta_456[0][0] = std::atan2(-R_36(2, 2), R_36(0, 2));
          theta_456[1][2] = std::atan2(-R_36(1, 1), R_36(1, 0));
          theta_456[0][2] = std::atan2(R_36(1, 1), -R_36(1, 0));
        }
        outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = 1;
        outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = 1;
      }

      for (int ind_wrist = 0; ind_wrist < 2; ind_wrist++) {
        double theta_1_ = theta_1[ind_arm] - jointOffset_[0];
        // while (theta_1_ > (double)M_PI) {
        //     theta_1_ -= 2.0 * (double)M_PI;
        // }

        // while (theta_1_ <= -(double)M_PI) {
        //     theta_1_ += 2.0 * (double)M_PI;
        // }
        outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist].q[0] =
            theta_1_;

        for (int i = 0; i < 2; i++) {
          double theta_23_ = theta_23[ind_elbow][i] - jointOffset_[1 + i];
          // while (theta_23_ > (double)M_PI) {
          //     theta_23_ -= 2.0 * (double)M_PI;
          // }
          // while (theta_23_ <= -(double)M_PI) {
          //     theta_23_ += 2.0 * (double)M_PI;
          // }
          outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist]
              .q[1 + i] = theta_23_;
        }

        for (int i = 0; i < 3; i++) {
          double theta_456_ = theta_456[ind_wrist][i] - jointOffset_[3 + i];
          // while (theta_456_ > (double)M_PI) {
          //     theta_456_ -= 2.0 * (double)M_PI;
          // }
          // while (theta_456_ <= -(double)M_PI) {
          //     theta_456_ += 2.0 * (double)M_PI;
          // }
          outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist]
              .q[3 + i] = theta_456_;
        }
      }
    }
  }

  return true;
}

bool KinematicSolver::IK_ParallelWrist(const ArmMotion::Pose6Dof_t& inputPose,
                                       const ArmMotion::Joint6Dof_t& lastJoint,
                                       ArmMotion::IKSolves_t& outputSolves) {
  // 具身智能使用较多的构型，234轴平行
  double theta_1[2]{};
  double theta_234[2][3]{};
  double theta_56[2][2]{};

  Eigen::Vector3d P_06_pos;      // yes
  Eigen::Quaterniond P_06_quat;  // yes
  Eigen::Vector3d P_05;
  Eigen::Matrix3d R_10;
  Eigen::Matrix3d R_06;  // yes
  Eigen::Vector3d P0_65;

  P_06_pos[0] = inputPose.x;
  P_06_pos[1] = inputPose.y;
  P_06_pos[2] = inputPose.z;

  // deg to rad
  P_06_quat = Eigen::Quaterniond(inputPose.quat.w, inputPose.quat.x,
                                 inputPose.quat.y, inputPose.quat.z);
  // quaternion to rotation
  P_06_quat.normalize();
  R_06 = P_06_quat.toRotationMatrix();

  // // 初始化解为上次的解，同步处理机械角度偏移
  // for (int i = 0; i < 2; i++) {
  //   theta_1[i] = lastJoint.q[0] + jointOffset_[0];
  //   theta_234[i][0] = lastJoint.q[1] + jointOffset_[1];
  //   theta_234[i][1] = lastJoint.q[2] + jointOffset_[2];
  //   theta_234[i][2] = lastJoint.q[3] + jointOffset_[3];
  //   theta_56[i][0] = lastJoint.q[4] + jointOffset_[4];
  //   theta_56[i][1] = lastJoint.q[5] + jointOffset_[5];
  // }

  // 切换参考参考坐标系为基座（纯旋转变换）
  P0_65 = R_06 * impl_->P_65;

  // 获取基座到手腕的位置向量
  // Eigen::Vector3d P_05;
  P_05.head<3>() = P_06_pos + P0_65;

  /* ---------------------------------J1------------------------------------- */
  if ((P_05[0] * P_05[0] + P_05[1] * P_05[1]) <= 1e-8) {
    /* 奇异位置，使用上次的解 */
    theta_1[0] = lastJoint.q[0] + jointOffset_[0];
    theta_1[1] = theta_1[0];
    for (int i = 0; i < 4; i++) {
      outputSolves.solveFlag[0 + i][0] = -1;
      outputSolves.solveFlag[4 + i][0] = -1;
    }
  } else {
    /* 正常解，左右肩两组解 */
    theta_1[0] = std::atan2(P_05[1], P_05[0]);
    theta_1[1] = std::atan2(-P_05[1], -P_05[0]);
    for (int i = 0; i < 4; i++) {
      outputSolves.solveFlag[0 + i][0] = 1;
      outputSolves.solveFlag[4 + i][0] = 1;
    }
  }

  /* ---------------------------------J234------------------------------------ */
  for (int ind_arm = 0; ind_arm < 2; ind_arm++) {
    double c1 = std::cos(theta_1[ind_arm]);
    double s1 = std::sin(theta_1[ind_arm]);

    // 构造 R_01 的逆矩阵
    R_10 << c1, s1, 0.0, -s1, c1, 0.0, 0.0, 0.0, 1.0;

    // 不同于球形，这里全部用向量表示
    Eigen::Vector3d P1_05, P1_02, P1_25;

    // 消除 J1 的影响，切换到 J1 的坐标系下进行后续计算
    P1_05 = R_10 * P_05;
    P1_02 = impl_->P_02;

    P1_25 = P1_05 - P1_02;

    // 算 q2、q3、q4 的和角
    Eigen::Matrix3d R_16;
    R_16 = R_10 * R_06;

    // 提取 q234 和角；在腕奇异附近（atan2 两输入同时趋近 0）退化到上次解以保持连续。
    double q234_sum_ref = lastJoint.q[1] + jointOffset_[1] +
                          lastJoint.q[2] + jointOffset_[2] +
                          lastJoint.q[3] + jointOffset_[3];
    double q234_y = -R_16(2, 2);
    double q234_x = R_16(0, 2);
    double q234_sum;
    
    if (std::hypot(q234_y, q234_x) < 1e-6) {
      q234_sum = q234_sum_ref;
    } else {
      q234_sum = std::atan2(q234_y, q234_x);
      while (q234_sum - q234_sum_ref > (double)M_PI) {
        q234_sum -= 2.0 * (double)M_PI;
      }
      while (q234_sum - q234_sum_ref < -(double)M_PI) {
        q234_sum += 2.0 * (double)M_PI;
      }
    }

    // 消除 L4 的影响
    double L2 = impl_->mdhParams_.a[2];
    double L3 = impl_->mdhParams_.a[3];
    double L4 = impl_->mdhParams_.a[4];

    double cq234 = std::cos(q234_sum);
    double sq234 = std::sin(q234_sum);

    if (cq234 > 1.0) {
      cq234 = 1.0;
    }
    if (cq234 < -1.0) {
      cq234 = -1.0;
    }

    if (sq234 > 1.0) {
      sq234 = 1.0;
    }
    if (sq234 < -1.0) {
      sq234 = -1.0;
    }

    double x = P1_25[0] - L4 * cq234;
    double z = P1_25[2] + L4 * sq234;

    // 余弦定理计算 q3 的余弦值，这里有一步隐式变换（-cos(c)=cos(q3)，c^2=x * x + z * z）
    double c3 = (x * x + z * z - L2 * L2 - L3 * L3) / (2 * L2 * L3);

    if (std::abs(c3) - 1.0 > 1e-4) {
      // 无解（太远或太近导致不可达）
      for (int i = 0; i < 2; i++) {
        outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 0;
        outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 0;
      }

      for (int ind_elbow = 0; ind_elbow < 2; ind_elbow++) {
        for (int ind_wrist = 0; ind_wrist < 2; ind_wrist++) {
          int solve_index = ind_arm * 4 + ind_elbow * 2 + ind_wrist;
          outputSolves.solveFlag[solve_index][2] = 0;
          for (int joint_index = 0; joint_index < 6; joint_index++) {
            outputSolves.solves[solve_index].q[joint_index] =
                lastJoint.q[joint_index];
          }
        }
      }
      continue;
    } else if (std::abs(std::abs(c3) - 1.0) < 1e-4) {
      // 奇异（肘伸直或折叠）
      double c3_clamped = c3;
      if (c3_clamped > 1.0) {
        c3_clamped = 1.0;
      }
      if (c3_clamped < -1.0) {
        c3_clamped = -1.0;
      }

      double q3;
      // 奇异处理：固定退化 q3
      if (c3_clamped >= 0.0) {
        q3 = 0.0;
      } else {
        q3 = (double)M_PI;
      }

      // 三角形退化
      double q2 = -std::atan2(z, x);

      // 第一组
      theta_234[0][0] = q2;
      theta_234[0][1] = q3;
      theta_234[0][2] = q234_sum - theta_234[0][0] - theta_234[0][1];

      // 第二组
      theta_234[1][0] = q2;
      theta_234[1][1] = -q3;
      theta_234[1][2] = q234_sum - theta_234[1][0] - theta_234[1][1];

      for (int i = 0; i < 2; i++) {
        outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = -1;
        outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = -1;
      }
    } else {
      // 正常解
      if (c3 > 1.0) {
        c3 = 1.0;
      }
      if (c3 < -1.0) {
        c3 = -1.0;
      }
      // 计算 q3（这里q3的两组结果对于肘上和肘下）
      theta_234[0][1] = std::atan2(std::sqrt(1 - c3 * c3), c3);
      theta_234[1][1] = std::atan2(-std::sqrt(1 - c3 * c3), c3);
      // 计算 q2
      theta_234[0][0] = -std::atan2(z, x) - std::atan2(L3 * std::sin(theta_234[0][1]),
                                                       L2 + L3 * std::cos(theta_234[0][1]));
      theta_234[1][0] = -std::atan2(z, x) - std::atan2(L3 * std::sin(theta_234[1][1]),
                                                       L2 + L3 * std::cos(theta_234[1][1]));
      // 计算 q4
      theta_234[0][2] = q234_sum - theta_234[0][0] - theta_234[0][1];
      theta_234[1][2] = q234_sum - theta_234[1][0] - theta_234[1][1];
      for (int i = 0; i < 2; i++) {
        outputSolves.solveFlag[0 + i + 4 * ind_arm][1] = 1;
        outputSolves.solveFlag[2 + i + 4 * ind_arm][1] = 1;
      }
    }
    /* ----------------------------------- J56 ------------------------------------- */
    for (int ind_elbow = 0; ind_elbow < 2; ind_elbow++) {
      Eigen::Matrix3d R_41, R_46;
      double q234 = theta_234[ind_elbow][0] + theta_234[ind_elbow][1] + theta_234[ind_elbow][2];
      double cq234 = std::cos(q234);
      double sq234 = std::sin(q234);

      R_41 << cq234, 0.0, -sq234, -sq234, 0.0, -cq234, 0.0, 1.0, 0.0;
      R_46 = R_41 * R_16;
      double s5 = R_46(0, 2);
      double c5 = R_46(2, 2);
      double s6 = R_46(1, 0);
      double c6 = R_46(1, 1);

      if (s5 > 1.0) {
        s5 = 1.0;
      }
      if (s5 < -1.0) {
        s5 = -1.0;
      }
      if (c5 > 1.0) {
        c5 = 1.0;
      }
      if (c5 < -1.0) {
        c5 = -1.0;
      }
      if (s6 > 1.0) {
        s6 = 1.0;
      }
      if (s6 < -1.0) {
        s6 = -1.0;
      }
      if (c6 > 1.0) {
        c6 = 1.0;
      }
      if (c6 < -1.0) {
        c6 = -1.0;
      }

      if (std::abs(s5) < 1e-4) {
        // 奇异时腕部两分支退化为同一组，避免镜像分支在奇异点附近跳变。
        outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = -1;
        outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = -1;

        // q5 在奇异处仅剩符号信息，按 c5 退化到 0 或 pi，并贴近上次解分支。
        double q5 = (c5 >= 0.0) ? 0.0 : (double)M_PI;
        double q5_ref = lastJoint.q[4] + jointOffset_[4];
        while (q5 - q5_ref > (double)M_PI) {
          q5 -= 2.0 * (double)M_PI;
        }
        while (q5 - q5_ref < -(double)M_PI) {
          q5 += 2.0 * (double)M_PI;
        }

        // q6 一般仍可由 R46 第二行直接反解；不可观测时回退到上次解。
        double q6 = std::atan2(s6, c6);

        theta_56[0][0] = q5;
        theta_56[1][0] = q5;
        theta_56[0][1] = q6;
        theta_56[1][1] = q6;
      } else {
        outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 0][2] = 1;
        outputSolves.solveFlag[4 * ind_arm + 2 * ind_elbow + 1][2] = 1;

        // 翻腕和不翻腕两组解
        theta_56[0][0] = std::atan2(s5, c5);
        theta_56[1][0] = std::atan2(-s5, -c5);
        theta_56[0][1] = std::atan2(s6, c6);
        theta_56[1][1] = std::atan2(-s6, -c6);
      }

      // 记录角度（减去关节偏移量）
      for (int ind_wrist = 0; ind_wrist < 2; ind_wrist++) {
        double theta_1_ = theta_1[ind_arm] - jointOffset_[0];
        outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist].q[0] = theta_1_;

        for (int i = 0; i < 3; i++) {
          double theta_234_ = theta_234[ind_elbow][i] - jointOffset_[1 + i];
          outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist].q[1 + i] = theta_234_;
        }

        for (int i = 0; i < 2; i++) {
          double theta_56_ = theta_56[ind_wrist][i] - jointOffset_[4 + i];
          outputSolves.solves[ind_arm * 4 + ind_elbow * 2 + ind_wrist]
              .q[4 + i] = theta_56_;
        }
      }
    }
  }
  return true;
}

void KinematicSolver::SetIKMode(IKMode mode) {
  ikMode_ = mode;
}

void KinematicSolver::SetToolPose(const ArmMotion::Pose6Dof_t& tool_pose) {
  impl_->T_6tool = PoseToTransform(tool_pose);
  impl_->T_tool6 = impl_->T_6tool.inverse();
  impl_->toolPoseEnabled = true;
}

void KinematicSolver::ClearToolPose() {
  impl_->T_6tool = Eigen::Matrix4d::Identity();
  impl_->T_tool6 = Eigen::Matrix4d::Identity();
  impl_->toolPoseEnabled = false;
}

bool KinematicSolver::IK(const ArmMotion::Pose6Dof_t& inputPose, const ArmMotion::Joint6Dof_t& lastJoint,
                         ArmMotion::IKSolves_t& outputSolves) {
  ArmMotion::Pose6Dof_t flange_pose = inputPose;
  if (impl_->toolPoseEnabled) {
    Eigen::Matrix4d T_0tcp = PoseToTransform(inputPose);
    Eigen::Matrix4d T_06 = T_0tcp * impl_->T_tool6;
    flange_pose = TransformToPose(T_06);
  }

  bool ok = false;
  if (ikMode_ == IKMode::SphericalWrist) {
    ok = IK_SphericalWrist(flange_pose, lastJoint, outputSolves);
  } else if (ikMode_ == IKMode::ParallelWrist) {
    ok = IK_ParallelWrist(flange_pose, lastJoint, outputSolves);
  } else {
    return false;
  }

  if (!ok) {
    return false;
  }

  return true;
}

KinematicSolver::~KinematicSolver() = default;