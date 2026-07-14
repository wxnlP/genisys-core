#pragma once

#include <vector>
#include "arm_type.h"

class Dynamics {
 public:
  struct Link {
    double mass{};        // 质量
    double com[3]{};      // 质心
    double pos[3]{};      // 连杆相对位置
    double inertia[9]{};  // 转动惯量
  };

  Dynamics(const ArmMotion::MDHParams& mdh_params, const std::vector<Link>& links, const std::vector<double>& g);

  void RNEA(const ArmMotion::Joint6Dof& theta, const ArmMotion::Joint6Dof& vel,
            const ArmMotion::Joint6Dof& accel, ArmMotion::Joint6Dof& torques);

  // 重力向量 G(q)
  void GravityVector(const ArmMotion::Joint6Dof& q, ArmMotion::Joint6Dof& G);

  // 科氏力+重力 C(q,v)v + G(q)
  void CoriolisGravity(const ArmMotion::Joint6Dof& q, const ArmMotion::Joint6Dof& v,
                       ArmMotion::Joint6Dof& CG);

  // 质量矩阵 M(q)
  void MassMatrix(const ArmMotion::Joint6Dof& q, double M[6][6]);

 private:
  std::vector<Link> links_;
  std::vector<double> g_;
  ArmMotion::MDHParams mdh_params_;
};