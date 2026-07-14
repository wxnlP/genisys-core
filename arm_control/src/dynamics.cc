#include "dynamics.h"
#include <Eigen/Dense>


Dynamics::Dynamics(const ArmMotion::MDHParams& mdh_params, const std::vector<Link>& links, const std::vector<double>& g)
    : mdh_params_(mdh_params), links_(links), g_(g) {
  // 处理 Link 连杆相对位置计算
  for (int i = 0; i < 6; ++i) {
    double a = mdh_params_.a[i];
    double d = mdh_params_.d[i];
    double alpha = mdh_params_.alpha[i];
    links_[i+1].pos[0] = a;
    links_[i+1].pos[1] = -std::sin(alpha) * d;
    links_[i+1].pos[2] = std::cos(alpha) * d;
  }
}


void Dynamics::RNEA(const ArmMotion::Joint6Dof& theta, const ArmMotion::Joint6Dof& vel,
                    const ArmMotion::Joint6Dof& accel, ArmMotion::Joint6Dof& torques) {
  // 实现 RNEA 算法

  // 初始化向量和矩阵变量
  std::vector<Eigen::Vector3d> dvc; // 连杆质心加速度
  std::vector<Eigen::Vector3d> dv;  // 连杆加速度
  std::vector<Eigen::Vector3d> dw;  // 关节角加速度
  std::vector<Eigen::Vector3d> w;   // 关节角速度
  std::vector<Eigen::Vector3d> f;   // 连杆力
  std::vector<Eigen::Vector3d> n;   // 连杆力矩
  std::vector<Eigen::Vector3d> F;   // 连杆质心力
  std::vector<Eigen::Vector3d> N;   // 连杆质心力矩
  std::vector<Eigen::Matrix3d> R;   // 连杆旋转矩阵

  // 计算旋转矩阵
  for (int i = 0; i < 6; ++i) {
    double sa = std::sin(mdh_params_.alpha[i]);
    double ca = std::cos(mdh_params_.alpha[i]);
    // 处理偏移角度
    double st = std::sin(theta.q[i]+mdh_params_.theta[i]);
    double ct = std::cos(theta.q[i]+mdh_params_.theta[i]);
    Eigen::Matrix3d R_i;
    R_i << ct,    -st,     0,
           st*ca,  ct*ca, -sa,
           st*sa,  ct*sa,  ca;
    R.push_back(R_i);
  }

  // 初始化基座
  dvc.push_back(Eigen::Vector3d::Zero()); // 基座质心加速度
  Eigen::Map<Eigen::Vector3d> g_vec(g_.data());
  dv.push_back(-g_vec);                   // 基座加速度
  dw.push_back(Eigen::Vector3d::Zero());  // 基座角加速度
  w.push_back(Eigen::Vector3d::Zero());   // 基座角速度
  F.push_back(Eigen::Vector3d::Zero());   // 基座质心力
  N.push_back(Eigen::Vector3d::Zero());   // 基座质心力矩

  // 旋转轴
  Eigen::Vector3d z_axis(0, 0, 1);

  // 前向递推计算每个连杆的角速度、角加速度和线加速度
  for (int i = 0; i < 6; ++i) {
    Eigen::Map<Eigen::Vector3d> com(links_[i+1].com);
    Eigen::Map<Eigen::Matrix3d> inertia(links_[i+1].inertia);
    Eigen::Map<Eigen::Vector3d> pos(links_[i+1].pos);
    double m = links_[i+1].mass;
    auto RT = R[i].transpose();

    w.push_back(RT * w[i] + vel.q[i] * z_axis);
    dw.push_back(RT * dw[i] + (RT * w[i]).cross(vel.q[i] * z_axis) + accel.q[i] * z_axis);
    dv.push_back(RT *(dw[i].cross(pos) + w[i].cross(w[i].cross(pos)) + dv[i]));
    dvc.push_back(dw[i+1].cross(com) + w[i+1].cross(w[i+1].cross(com)) + dv[i+1]);
    F.push_back(m * dvc[i+1]);
    N.push_back(inertia * dw[i+1] + w[i+1].cross(inertia * w[i+1]));
  }

  // 后向递推计算每个连杆的力和力矩
  ArmMotion::Joint6Dof tau;
  f.resize(7);
  n.resize(7);
  Eigen::Map<Eigen::Vector3d> com(links_[6].com);
  f[6] = F[6];
  n[6] = N[6] + com.cross(F[6]);
  tau.q[5] = n[6].dot(z_axis);
  for (int i = 5; i > 0; --i) {
    Eigen::Map<Eigen::Vector3d> com(links_[i].com);
    Eigen::Map<Eigen::Vector3d> pos(links_[i+1].pos);
    f[i] = R[i] * f[i+1] + F[i];
    n[i] = R[i] * n[i+1] + N[i] + com.cross(F[i]) + pos.cross(R[i] * f[i+1]);
    tau.q[i-1] = n[i].dot(z_axis);
  }
  torques = tau;
}

void Dynamics::GravityVector(const ArmMotion::Joint6Dof& q, ArmMotion::Joint6Dof& G) {
  ArmMotion::Joint6Dof zero;
  RNEA(q, zero, zero, G);
}

void Dynamics::CoriolisGravity(const ArmMotion::Joint6Dof& q,
                                const ArmMotion::Joint6Dof& v,
                                ArmMotion::Joint6Dof& CG) {
  ArmMotion::Joint6Dof zero;
  RNEA(q, v, zero, CG);
}

void Dynamics::MassMatrix(const ArmMotion::Joint6Dof& q, double M[6][6]) {
  ArmMotion::Joint6Dof zero_v, unit_a, column;

  for (int i = 0; i < 6; ++i) {
    unit_a.q[i] = 1.0;
    RNEA(q, zero_v, unit_a, column);
    for (int j = 0; j < 6; ++j)
      M[j][i] = column.q[j];
    unit_a.q[i] = 0.0;
  }
}