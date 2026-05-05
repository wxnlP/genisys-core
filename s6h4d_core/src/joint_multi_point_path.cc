#include "joint_multi_point_path.h"

#include <algorithm>
#include <cmath>

/* 关节欧式距离计算 */
double MultiPointJointPath::JointDistance(const ArmMotion::Joint6Dof_t& p1,
                                          const ArmMotion::Joint6Dof_t& p2) const {
  double sum = 0.0;
  for (int i = 0; i < 6; ++i) {
    double d = p1.theta[i] - p2.theta[i];
    sum += d * d;
  }
  return std::sqrt(sum);
}

// ─────────────────────────────────────────────────────────────────────────────
// 构造 & 预计算
// ─────────────────────────────────────────────────────────────────────────────

MultiPointJointPath::MultiPointJointPath(std::vector<ArmMotion::Joint6Dof_t> points, double r)
    : points_(std::move(points)), r_(r) {
  Precompute();
}

void MultiPointJointPath::Precompute() {
  const int n = static_cast<int>(points_.size());
  if (n <= 2) {
    total_length_ = 0.0;
    return;
  }

  seg_lengths_.resize(n - 1);
  cum_lengths_.resize(n, 0.0);
  blend_radii_.resize(n, 0.0);

  // 各线段长度及各路径点累计弧长
  for (int i = 0; i < n - 1; ++i) {
    seg_lengths_[i] = JointDistance(points_[i], points_[i + 1]);
    cum_lengths_[i + 1] = cum_lengths_[i] + seg_lengths_[i];
  }
  total_length_ = cum_lengths_[n - 1];

  // 中间路径点的有效融合半径（端点保持 0）
  for (int j = 1; j < n - 1; ++j) {
    double max_r = std::min(seg_lengths_[j - 1] * 0.5, seg_lengths_[j] * 0.5);
    blend_radii_[j] = std::min(r_, max_r);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 路径评估
// ─────────────────────────────────────────────────────────────────────────────

// 抛物线融合公式推导（以路径点 P[j] 为融合中心，u ∈ [-r, r]）：
//
//   d_in  = (P[j] - P[j-1]) / L[j-1]   入射单位方向
//   d_out = (P[j+1] - P[j]) / L[j]     出射单位方向
//
//   p(u) = P[j] + r*(d_out - d_in)/4
//                + (d_in + d_out)/2 * u
//                + (d_out - d_in)/(4*r) * u^2
//
// 验证：p(-r) = P[j] - r*d_in  ✓
//        p(+r) = P[j] + r*d_out ✓
//        p'(-r) = d_in  ✓   p'(+r) = d_out ✓   （G1 连续）
//
// 切向量（∂p/∂u 再乘以 total_length_ 换算到 ds）：
//   dp/du = (d_in + d_out)/2 + (d_out - d_in)/(2*r) * u

void MultiPointJointPath::EvaluatePos(double s, ArmMotion::Joint6Dof_t& point) const {
  const int n = static_cast<int>(points_.size());

  // 只考虑2点以上的情况
  if (n <= 2) { return; }

  s = std::max(0.0, std::min(1.0, s));
  // s 对应的弧长位置 arc
  const double arc = s * total_length_;

  // 检查是否处于某中间路径点的融合区
  for (int j = 1; j < n - 1; ++j) {
    const double r = blend_radii_[j]; // 端点为0，所以索引从1开始
    if (r <= 0.0) { continue; }

    const double arc_j = cum_lengths_[j];
    // 跳过非融合区段
    if (arc < arc_j - r || arc > arc_j + r) { continue; }

    const double u = arc - arc_j;  // u ∈ [-r, r]

    const double L_in  = seg_lengths_[j - 1];
    const double L_out = seg_lengths_[j];

    // 计算各关节分量
    for (int k = 0; k < 6; ++k) {
      const double d_in  = (points_[j].theta[k] - points_[j - 1].theta[k]) / L_in;
      const double d_out = (points_[j + 1].theta[k] - points_[j].theta[k]) / L_out;

      point.theta[k] = points_[j].theta[k]
                       + r * (d_out - d_in) * 0.25
                       + (d_in + d_out) * 0.5 * u
                       + (d_out - d_in) / (4.0 * r) * u * u;

      // 切向量 d(pos)/ds = dp/du * L_total
      last_tangent_.theta[k] =
          ((d_in + d_out) * 0.5 + (d_out - d_in) / (2.0 * r) * u)
          * total_length_;
    }
    return;
  }

  // 找到 arc 所在线段 seg
  int seg = n - 2;  // 默认：最后一段
  for (int i = 0; i < n - 1; ++i) {
    if (arc <= cum_lengths_[i + 1]) {
      seg = i;
      break;
    }
  }

  const double L = seg_lengths_[seg];
  // 不应该发生，但以防万一（如数值误差导致 L=0），直接返回线段起点
  if (L <= 0.0) {
    point = points_[seg];
    for (int k = 0; k < 6; ++k) { last_tangent_.theta[k] = 0.0; }
    return;
  }

  const double t = (arc - cum_lengths_[seg]) / L;  // 百分比
  for (int k = 0; k < 6; ++k) {
    const double delta = points_[seg + 1].theta[k] - points_[seg].theta[k];
    point.theta[k] = points_[seg].theta[k] + t * delta;
    // 切向量 = d(pos)/ds = delta/L * L_total
    last_tangent_.theta[k] = delta * total_length_ / L;
  }
}

void MultiPointJointPath::EvaluateTangent(ArmMotion::Joint6Dof_t& tangent) const {
  tangent = last_tangent_;
}
