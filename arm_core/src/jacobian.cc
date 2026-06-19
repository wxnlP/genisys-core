#include "jacobian.h"

#include <Eigen/Dense>

class Jacobian6Dof::Impl {
 public:
  // MDH params
  ArmMotion::MDHParams mdhParams_{};

  // 雅可比
  Eigen::Matrix<double, 6, 6> current_jacobian_;
  Eigen::Matrix<double, 6, 6> last_jacobian_;
  Eigen::Matrix<double, 6, 6> jacobian_dot_;
  Eigen::Matrix<double, 6, 1> current_dq_;

  explicit Impl(const ArmMotion::MDHParams& mdh_params)
      : mdhParams_(mdh_params) {}
};

Jacobian6Dof::Jacobian6Dof(const ArmMotion::MDHParams& mdh_params)
    : impl_(std::make_unique<Impl>(mdh_params)) {
  jointOffset_[0] = mdh_params.theta[0];
  jointOffset_[1] = mdh_params.theta[1];
  jointOffset_[2] = mdh_params.theta[2];
  jointOffset_[3] = mdh_params.theta[3];
  jointOffset_[4] = mdh_params.theta[4];
  jointOffset_[5] = mdh_params.theta[5];
}

void Jacobian6Dof::Update(const ArmMotion::Joint6Dof& inputJoint, double dt) {
  // 齐次矩阵
  Eigen::Matrix4d T_02, T_03, T_04, T_05, T_06;
  Eigen::Matrix4d T[6];
  // 雅可比构造参数
  Eigen::Vector3d z[6];
  Eigen::Vector3d p[6];
  Eigen::Vector3d pe;
  double theta[6];

  for (int i = 0; i < 6; i++) {
    theta[i] = inputJoint.q[i] + jointOffset_[i];

    double st = std::sin(theta[i]);
    double ct = std::cos(theta[i]);
    double sa = std::sin(impl_->mdhParams_.alpha[i]);
    double ca = std::cos(impl_->mdhParams_.alpha[i]);
    double a = impl_->mdhParams_.a[i];
    double d = impl_->mdhParams_.d[i];

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

bool Jacobian6Dof::ForwardVelocity(const ArmMotion::Joint6Dof& inputVel,
                                   ArmMotion::CartesianTwist& outputVel) {
  Eigen::Matrix<double, 6, 1> jointVel;
  jointVel << inputVel.q[0], inputVel.q[1], 
              inputVel.q[2], inputVel.q[3], 
              inputVel.q[4], inputVel.q[5];
  Eigen::Matrix<double, 6, 1> cartesianVel = impl_->current_jacobian_ * jointVel;

  outputVel.linear.x = cartesianVel[0];
  outputVel.linear.y = cartesianVel[1];
  outputVel.linear.z = cartesianVel[2];
  outputVel.angular.x = cartesianVel[3];
  outputVel.angular.y = cartesianVel[4];
  outputVel.angular.z = cartesianVel[5];

  return true;
}

bool Jacobian6Dof::InverseVelocity(const ArmMotion::CartesianTwist& inputVel,
                                   ArmMotion::Joint6Dof& outputVel,
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

  Eigen::Map<Eigen::Matrix<double, 6, 1>> outMap(&outputVel.q[0]);
  outMap = dqResult;
  return true;
}

bool Jacobian6Dof::ForwardAccel(const ArmMotion::Joint6Dof& inputAcc,
                                ArmMotion::CartesianTwist& outputAcc) {
  Eigen::Matrix<double, 6, 1> jointAcc;
  jointAcc << inputAcc.q[0], inputAcc.q[1], inputAcc.q[2],
              inputAcc.q[3], inputAcc.q[4], inputAcc.q[5];
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

bool Jacobian6Dof::InverseAccel(const ArmMotion::CartesianTwist& inputAcc,
                                ArmMotion::Joint6Dof& outputAcc,
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

  Eigen::Map<Eigen::Matrix<double, 6, 1>> outMap(&outputAcc.q[0]);
  outMap = ddqResult;
  return true;
}

Jacobian6Dof::~Jacobian6Dof() = default;