#pragma once

#include <vector>

#include "joint_path_base.h"

// 关节空间多点路径，在中间路径点处使用抛物线融合（parabolic blend）。
//
// 参数说明：
//   points : 路径点序列（至少 2 个）
//   r      : 融合半径（关节空间欧氏距离，单位与关节角度一致）；
//            自动裁剪为相邻线段长度的一半，确保不相互重叠。
//
// 路径参数 s ∈ [0, 1]，按累计弧长均匀化。
// EvaluateTangent() 返回上一次 EvaluatePos 调用处的 d(pos)/ds，
// 幅值约等于总弧长，供 JointTrajectory 速度/加速度缩放使用。
class MultiPointJointPath : public JointPathBase {
 public:
  MultiPointJointPath(std::vector<ArmMotion::Joint6Dof> points, double r);
  ~MultiPointJointPath() override = default;

  void EvaluatePos(double s, ArmMotion::Joint6Dof& point) const override;
  void EvaluateTangent(ArmMotion::Joint6Dof& tangent) const override;
  double GetTotalLength() const { return total_length_; }
 private:
  double JointDistance(const ArmMotion::Joint6Dof& p1, const ArmMotion::Joint6Dof& p2) const;
  void Precompute();

  std::vector<ArmMotion::Joint6Dof> points_;
  double r_;

  // 预计算数据
  std::vector<double> seg_lengths_;  // 关节欧式距离（n-1）
  std::vector<double> cum_lengths_;  // 累计弧长（n，cum_lengths_[0] = 0）
  std::vector<double> blend_radii_;  // 各路径点处有效融合半径（端点为 0）
  double total_length_{0.0};         // 总弧长 = cum_lengths_[n-1]    

  // 缓存上一次 EvaluatePos 求得的切向量，供 EvaluateTangent 使用
  mutable ArmMotion::Joint6Dof last_tangent_{};
};