#pragma once

#include <memory>
#include <vector>

#include "cartesian_path_base.h"

struct CartesianMultiPointPathImpl;

// 笛卡尔空间多点路径，在中间路径点处使用圆弧融合（circular arc blend）。
//
// 位置：相邻路径点间直线插值，中间路径点处用真圆弧平滑过渡（G1 连续）。
// 姿态：各路径点间球面线性插值（Slerp），按弧长比例分配。
//
// 参数说明：
//   points : 路径点序列（至少 2 个），位置单位与 r 一致（通常 m）
//   r      : 融合半径（笛卡尔距离），自动裁剪为相邻线段长度的一半
//
// 路径参数 s ∈ [0, 1]，按累计弧长均匀化。
// GetTotalLength() 返回路径总弧长，供 LSPBTimeScaler 规划使用。
class CartesianMultiPointPath : public CartesianPathBase {
 public:
  CartesianMultiPointPath(std::vector<ArmMotion::Pose6Dof> points, double r);
  ~CartesianMultiPointPath() override;

  void EvaluatePos(double s, ArmMotion::Pose6Dof& point) const override;
  void EvaluateTangent(ArmMotion::Vector3d& pos, ArmMotion::Vector3d& omega) const override;

  double GetTotalLength() const;

 private:
  std::unique_ptr<CartesianMultiPointPathImpl> impl_;
};