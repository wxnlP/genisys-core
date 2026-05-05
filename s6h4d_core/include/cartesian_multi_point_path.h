#pragma once

#include <vector>

#include <Eigen/Dense>
#include "cartesian_path_base.h"

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
  CartesianMultiPointPath(std::vector<ArmMotion::Pose6Dof_t> points, double r);
  ~CartesianMultiPointPath() override = default;

  void EvaluatePos(double s, ArmMotion::Pose6Dof_t& point) const override;
  void EvaluateTangent(ArmMotion::Vector3d& pos, ArmMotion::Vector3d& omega) const override;

  double GetTotalLength() const { return total_length_; }

 private:
  void Precompute();

  // 中间路径点 P[j] 处的圆弧融合数据（blends_[j-1] 对应路径点 j）
  struct BlendArc {
    double r_eff{0.0};     // 有效融合半径 (m)
    double beta{0.0};      // 转向角 (rad)
    double d_cut{0.0};     // 切点到 P[j] 的距离
    double arc_len{0.0};   // 圆弧弧长 = r_eff * beta
    double cum_arc_A{0.0}; // 路径累计弧长：圆弧起点 A[j]
    Eigen::Vector3d O{};   // 圆心
    Eigen::Vector3d e_in{};// 入射单位方向（= R_y 基向量）
    Eigen::Vector3d n1{};  // 垂直于 e_in 的内法向量（R_x = -n1）
  };

  std::vector<ArmMotion::Pose6Dof_t> points_;
  double r_;

  int n_{0};
  std::vector<double> seg_lengths_;         // L[i] = |P[i+1]-P[i]|
  std::vector<Eigen::Vector3d> seg_dirs_;   // e[i] = normalize(P[i+1]-P[i])
  std::vector<Eigen::Vector3d> pos_;        // 各路径点的 3D 位置
  std::vector<Eigen::Quaterniond> quats_;   // 各路径点的四元数（保证最短路径）
  std::vector<BlendArc> blends_;            // 大小 n-2，blends_[j-1] 对应 P[j]

  // 每段直线部分的累计弧长范围及起点位置
  std::vector<double> straight_cum_start_;        // 直线段 i 的弧长起点
  std::vector<double> straight_cum_end_;          // 直线段 i 的弧长终点
  std::vector<Eigen::Vector3d> straight_start_pos_; // 直线段 i 的实际起点（切点 B 或 P[0]）

  // 姿态插值路标点弧长（端点 0/total，中间为各圆弧弧长中点）
  std::vector<double> ori_arcs_;

  double total_length_{0.0};

  // 缓存上一次 EvaluatePos 求得的切向量，供 EvaluateTangent 使用
  mutable ArmMotion::Vector3d last_pos_tangent_{};
  mutable ArmMotion::Vector3d last_ang_tangent_{};
};