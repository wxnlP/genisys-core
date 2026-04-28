#include "jacobian.h"

#include <Eigen/Dense>

class Jacobian6Dof::Impl {
 public:
  // MDH params
  ArmMotion::MDHConfig_t mdhConfig_{};

  // 雅可比
  Eigen::Matrix<double, 6, 6> current_jacobian_;
  Eigen::Matrix<double, 6, 6> last_jacobian_;
  Eigen::Matrix<double, 6, 6> jacobian_dot_;
  Eigen::Matrix<double, 6, 1> current_dq_;
  Eigen::Matrix<double, 6, 4> MDH_Matrix_;

  explicit Impl(const ArmMotion::MDHConfig_t& mdh_config)
      : mdhConfig_(mdh_config) {
    MDH_Matrix_ << 0.0000, 0.00, mdhConfig_.L_BASE, 0.0, -(double)M_PI_2,
        mdhConfig_.D_BASE, 0.00, 0.0, 0.0000, mdhConfig_.L_UPPERARM, 0.00, 0.0,
        -(double)M_PI_2, mdhConfig_.D_ELBOW, mdhConfig_.L_FOREARM, 0.0,
        (double)M_PI_2, 0.00, 0.00, 0.0, -(double)M_PI_2, 0.00,
        mdhConfig_.L_WRIST, 0.0;
  }
};

Jacobian6Dof::Jacobian6Dof(const ArmMotion::MDHConfig_t& mdh_config)
    : impl_(std::make_unique<Impl>(mdh_config)) {
  jointOffset_[0] = 0.0;
  jointOffset_[1] = -(double)M_PI_2;
  // jointOffset_[1] = 0.0f;
  jointOffset_[2] = 0.0;
  jointOffset_[3] = 0.0;
  jointOffset_[4] = 0.0;
  jointOffset_[5] = 0.0;
}

void Jacobian6Dof::Update(const ArmMotion::Joint6Dof_t& inputJoint, double dt) {
  // 齐次矩阵
  Eigen::Matrix4d T_02, T_03, T_04, T_05, T_06;
  Eigen::Matrix4d T[6];
  // 雅可比构造参数
  Eigen::Vector3d z[6];
  Eigen::Vector3d p[6];
  Eigen::Vector3d pe;
  double theta[6];

  for (int i = 0; i < 6; i++) {
    theta[i] = inputJoint.theta[i] + jointOffset_[i];

    double st = std::sin(theta[i]);
    double ct = std::cos(theta[i]);
    double sa = std::sin(impl_->MDH_Matrix_(i, 0));
    double ca = std::cos(impl_->MDH_Matrix_(i, 0));
    double a = impl_->MDH_Matrix_(i, 1);
    double d = impl_->MDH_Matrix_(i, 2);

    T[i] << ct, -st, 0.0, a, st * ca, ct * ca, -sa, -sa * d, st * sa, ct * sa,
        ca, ca * d, 0.0, 0.0, 0.0, 1.0;
  }

  T_02 = T[0] * T[1];
  T_03 = T_02 * T[2];
  T_04 = T_03 * T[3];
  T_05 = T_04 * T[4];
  T_06 = T_05 * T[5];

  p[0] = T[0].block<3, 1>(0, 3);
  p[1] = T_02.block<3, 1>(0, 3);
  p[2] = T_03.block<3, 1>(0, 3);
  p[3] = T_04.block<3, 1>(0, 3);
  p[4] = T_05.block<3, 1>(0, 3);
  p[5] = T_06.block<3, 1>(0, 3);
  pe = T_06.block<3, 1>(0, 3);
  z[0] = T[0].block<3, 1>(0, 2);
  z[1] = T_02.block<3, 1>(0, 2);
  z[2] = T_03.block<3, 1>(0, 2);
  z[3] = T_04.block<3, 1>(0, 2);
  z[4] = T_05.block<3, 1>(0, 2);
  z[5] = T_06.block<3, 1>(0, 2);

  // 构建雅可比矩阵
  impl_->last_jacobian_ = impl_->current_jacobian_;
  for (int i = 0; i < 6; i++) {
    // 线速度
    impl_->current_jacobian_.block<3, 1>(0, i) = z[i].cross(pe - p[i]);
    // 角速度
    impl_->current_jacobian_.block<3, 1>(3, i) = z[i];
  }
  if (dt > 1e-6) {
    impl_->jacobian_dot_ =
        (impl_->current_jacobian_ - impl_->last_jacobian_) / dt;
  } else {
    impl_->jacobian_dot_.setZero();
  }
}

bool Jacobian6Dof::ForwardVelocity(const ArmMotion::Joint6Dof_t& inputVel,
                                   ArmMotion::CartesianTwist_t& outputVel) {
  Eigen::Matrix<double, 6, 1> jointVel;
  jointVel << inputVel.theta[0], inputVel.theta[1], 
              inputVel.theta[2], inputVel.theta[3], 
              inputVel.theta[4], inputVel.theta[5];
  Eigen::Matrix<double, 6, 1> cartesianVel = impl_->current_jacobian_ * jointVel;

  outputVel.linear.x = cartesianVel[0];
  outputVel.linear.y = cartesianVel[1];
  outputVel.linear.z = cartesianVel[2];
  outputVel.angular.x = cartesianVel[3];
  outputVel.angular.y = cartesianVel[4];
  outputVel.angular.z = cartesianVel[5];

  return true;
}

bool Jacobian6Dof::InverseVelocity(const ArmMotion::CartesianTwist_t& inputVel,
                                   ArmMotion::Joint6Dof_t& outputVel,
                                   double damping) {
  Eigen::Matrix<double, 6, 1> cartesianVel;
  cartesianVel << inputVel.linear.x, inputVel.linear.y,inputVel.linear.z,
                  inputVel.angular.x, inputVel.angular.y, inputVel.angular.z;
  // 求雅可比伪逆
  Eigen::Matrix<double, 6, 6> J = impl_->current_jacobian_;
  Eigen::Matrix<double, 6, 6> JT = J.transpose();
  Eigen::Matrix<double, 6, 6> JJT = J * JT;

  for (int i = 0; i < 6; ++i) {
    JJT(i, i) += (damping * damping);
  }

  // 使用LLDT计算代替求逆
  Eigen::Matrix<double, 6, 1> x = JJT.ldlt().solve(cartesianVel);
  Eigen::Matrix<double, 6, 1> dqResult = JT * x;
  impl_->current_dq_ = dqResult;

  Eigen::Map<Eigen::Matrix<double, 6, 1>> outMap(&outputVel.theta[0]);
  outMap = dqResult;
  return true;
}

bool Jacobian6Dof::ForwardAccel(const ArmMotion::Joint6Dof_t& inputAcc,
                                ArmMotion::CartesianTwist_t& outputAcc) {
  Eigen::Matrix<double, 6, 1> jointAcc;
  jointAcc << inputAcc.theta[0], inputAcc.theta[1], inputAcc.theta[2],
              inputAcc.theta[3], inputAcc.theta[4], inputAcc.theta[5];
  Eigen::Matrix<double, 6, 1> cartesianAcc =
      impl_->current_jacobian_ * jointAcc +
      impl_->jacobian_dot_ * impl_->current_dq_;

  outputAcc.linear.x = cartesianAcc[0];
  outputAcc.linear.y = cartesianAcc[1];
  outputAcc.linear.z = cartesianAcc[2];
  outputAcc.angular.x = cartesianAcc[3];
  outputAcc.angular.y = cartesianAcc[4];
  outputAcc.angular.z = cartesianAcc[5];

  return true;
}

bool Jacobian6Dof::InverseAccel(const ArmMotion::CartesianTwist_t& inputAcc,
                                ArmMotion::Joint6Dof_t& outputAcc,
                                double damping) {
  Eigen::Matrix<double, 6, 1> cartesianAcc;
  cartesianAcc << inputAcc.linear.x, inputAcc.linear.y, inputAcc.linear.z,
                  inputAcc.angular.x, inputAcc.angular.y, inputAcc.angular.z;
  // 求雅可比伪逆
  Eigen::Matrix<double, 6, 6> J = impl_->current_jacobian_;
  Eigen::Matrix<double, 6, 6> JT = J.transpose();
  Eigen::Matrix<double, 6, 6> JJT = J * JT;

  for (int i = 0; i < 6; ++i) {
    JJT(i, i) += (damping * damping);
  }

  // 考虑雅可比矩阵的时间变化率
  Eigen::Matrix<double, 6, 1> bias_acc =
      cartesianAcc - impl_->jacobian_dot_ * impl_->current_dq_;

  // 使用LLDT计算代替求逆
  Eigen::Matrix<double, 6, 1> x = JJT.ldlt().solve(bias_acc);
  Eigen::Matrix<double, 6, 1> ddqResult = JT * x;

  Eigen::Map<Eigen::Matrix<double, 6, 1>> outMap(&outputAcc.theta[0]);
  outMap = ddqResult;
  return true;
}

Jacobian6Dof::~Jacobian6Dof() = default;