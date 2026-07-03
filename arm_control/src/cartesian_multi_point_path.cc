#include "cartesian_multi_point_path.h"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Impl 定义（所有依赖 Eigen 的实现细节全部藏在这里）
// ─────────────────────────────────────────────────────────────────────────────

struct CartesianMultiPointPathImpl {
  struct BlendArc {
    double r_eff{0.0};
    double beta{0.0};
    double d_cut{0.0};
    double arc_len{0.0};
    double cum_arc_A{0.0};
    Eigen::Vector3d O{};
    Eigen::Vector3d e_in{};
    Eigen::Vector3d n1{};
  };

  std::vector<ArmMotion::Pose6Dof> points_;
  double r_;

  int n_{0};
  std::vector<double> seg_lengths_;
  std::vector<Eigen::Vector3d> seg_dirs_;
  std::vector<Eigen::Vector3d> pos_;
  std::vector<Eigen::Quaterniond> quats_;
  std::vector<BlendArc> blends_;

  std::vector<double> straight_cum_start_;
  std::vector<double> straight_cum_end_;
  std::vector<Eigen::Vector3d> straight_start_pos_;

  std::vector<double> ori_arcs_;

  double total_length_{0.0};

  mutable ArmMotion::Vector3d last_pos_tangent_{};
  mutable ArmMotion::Vector3d last_ang_tangent_{};
};

// ─────────────────────────────────────────────────────────────────────────────
// 姿态插值辅助（内部）
// ─────────────────────────────────────────────────────────────────────────────

static void SampleOrientation(
    double arc,
    const std::vector<double>& ori_arcs,
    const std::vector<Eigen::Quaterniond>& quats,
    double total_length,
    ArmMotion::Pose6Dof& out_point,
    ArmMotion::Vector3d& out_ang_tangent) {
  const int n = static_cast<int>(ori_arcs.size());

  int k = n - 2;
  for (int i = 0; i < n - 1; ++i) {
    if (arc <= ori_arcs[i + 1]) { k = i; break; }
  }

  const double span = ori_arcs[k + 1] - ori_arcs[k];
  const double t    = (span > 1e-12) ? (arc - ori_arcs[k]) / span : 1.0;
  const Eigen::Quaterniond q = quats[k].slerp(t, quats[k + 1]);

  out_point.quat.w = q.w();
  out_point.quat.x = q.x();
  out_point.quat.y = q.y();
  out_point.quat.z = q.z();

  Eigen::Quaterniond dq = quats[k].inverse() * quats[k + 1];
  if (dq.w() < 0.0) dq.coeffs() *= -1.0;
  Eigen::AngleAxisd aa(dq);
  const double scale = (span > 1e-12) ? (total_length / span) : 0.0;
  Eigen::Vector3d omega = aa.axis() * aa.angle() * scale;
  out_ang_tangent = {omega.x(), omega.y(), omega.z()};
}

// ─────────────────────────────────────────────────────────────────────────────
// 预计算（自由函数，避免暴露到 header）
// ─────────────────────────────────────────────────────────────────────────────

static void Precompute(CartesianMultiPointPathImpl& impl) {
  impl.n_ = static_cast<int>(impl.points_.size());
  if (impl.n_ < 2) {
    impl.total_length_ = 0.0;
    return;
  }

  // ── Step 1: 提取位置和四元数 ─────────────────────────────────────────────
  impl.pos_.resize(impl.n_);
  impl.quats_.resize(impl.n_);
  for (int i = 0; i < impl.n_; ++i) {
    impl.pos_[i] = {impl.points_[i].x, impl.points_[i].y, impl.points_[i].z};
    impl.quats_[i] = Eigen::Quaterniond(impl.points_[i].quat.w, impl.points_[i].quat.x,
                                        impl.points_[i].quat.y, impl.points_[i].quat.z).normalized();
  }
  for (int i = 1; i < impl.n_; ++i) {
    if (impl.quats_[i - 1].dot(impl.quats_[i]) < 0.0) impl.quats_[i].coeffs() *= -1.0;
  }

  // ── Step 2: 各线段方向与长度 ─────────────────────────────────────────────
  impl.seg_dirs_.resize(impl.n_ - 1);
  impl.seg_lengths_.resize(impl.n_ - 1);
  for (int i = 0; i < impl.n_ - 1; ++i) {
    Eigen::Vector3d delta = impl.pos_[i + 1] - impl.pos_[i];
    impl.seg_lengths_[i] = delta.norm();
    if (impl.seg_lengths_[i] > 1e-12) {
      impl.seg_dirs_[i] = delta / impl.seg_lengths_[i];
    } else {
      impl.seg_dirs_[i] = Eigen::Vector3d::UnitX();
    }
  }

  // ── Step 3: 各中间路径点的圆弧融合参数 ───────────────────────────────────
  impl.blends_.resize(impl.n_ - 2);
  for (int j = 1; j <= impl.n_ - 2; ++j) {
    auto& b = impl.blends_[j - 1];
    const Eigen::Vector3d& e_in  = impl.seg_dirs_[j - 1];
    const Eigen::Vector3d& e_out = impl.seg_dirs_[j];

    const double dot  = std::max(-1.0, std::min(1.0, e_in.dot(e_out)));
    const double beta = std::acos(dot);

    if (beta < 1e-6) { b = {}; continue; }

    double d = impl.r_ * std::tan(beta * 0.5);
    d = std::min({d, impl.seg_lengths_[j - 1] * 0.499, impl.seg_lengths_[j] * 0.499});
    const double r_eff = d / std::tan(beta * 0.5);

    Eigen::Vector3d n1 = e_out - dot * e_in;
    if (n1.norm() < 1e-12) { b = {}; continue; }
    n1.normalize();

    b.r_eff   = r_eff;
    b.beta    = beta;
    b.d_cut   = d;
    b.arc_len = r_eff * beta;
    b.e_in    = e_in;
    b.n1      = n1;
    b.O       = impl.pos_[j] - d * e_in + r_eff * n1;
    b.cum_arc_A = 0.0;
  }

  // ── Step 4: 累计弧长 ──────────────────────────────────────────────────────
  impl.straight_cum_start_.resize(impl.n_ - 1);
  impl.straight_cum_end_.resize(impl.n_ - 1);
  impl.straight_start_pos_.resize(impl.n_ - 1);

  double arc = 0.0;
  for (int i = 0; i < impl.n_ - 1; ++i) {
    const double d_start = (i > 0)          ? impl.blends_[i - 1].d_cut : 0.0;
    const double d_end   = (i < impl.n_ - 2) ? impl.blends_[i].d_cut     : 0.0;

    impl.straight_start_pos_[i] = impl.pos_[i] + d_start * impl.seg_dirs_[i];

    impl.straight_cum_start_[i] = arc;
    arc += impl.seg_lengths_[i] - d_start - d_end;
    impl.straight_cum_end_[i] = arc;

    if (i < impl.n_ - 2) {
      impl.blends_[i].cum_arc_A = arc;
      arc += impl.blends_[i].arc_len;
    }
  }
  impl.total_length_ = arc;

  // ── Step 5: 姿态插值路标弧长 ─────────────────────────────────────────────
  impl.ori_arcs_.resize(impl.n_);
  impl.ori_arcs_[0]           = 0.0;
  impl.ori_arcs_[impl.n_ - 1] = impl.total_length_;
  for (int j = 1; j <= impl.n_ - 2; ++j) {
    const auto& b = impl.blends_[j - 1];
    impl.ori_arcs_[j] = (b.arc_len > 0.0)
                            ? b.cum_arc_A + b.arc_len * 0.5
                            : impl.straight_cum_end_[j - 1];
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 构造 & 析构
// ─────────────────────────────────────────────────────────────────────────────

CartesianMultiPointPath::CartesianMultiPointPath(
    std::vector<ArmMotion::Pose6Dof> points, double r)
    : impl_(std::make_unique<CartesianMultiPointPathImpl>()) {
  impl_->points_ = std::move(points);
  impl_->r_ = r;
  Precompute(*impl_);
}

CartesianMultiPointPath::~CartesianMultiPointPath() = default;

double CartesianMultiPointPath::GetTotalLength() const {
  return impl_->total_length_;
}

// ─────────────────────────────────────────────────────────────────────────────
// 路径采样
// ─────────────────────────────────────────────────────────────────────────────

void CartesianMultiPointPath::EvaluatePos(double s,
                                          ArmMotion::Pose6Dof& point) const {
  if (impl_->n_ < 2) return;

  s = std::max(0.0, std::min(1.0, s));
  const double arc = s * impl_->total_length_;

  // ── 检查圆弧融合区 ───────────────────────────────────────────────────────
  for (int j = 1; j <= impl_->n_ - 2; ++j) {
    const auto& b = impl_->blends_[j - 1];
    if (b.arc_len <= 0.0) continue;
    if (arc < b.cum_arc_A) break;
    if (arc > b.cum_arc_A + b.arc_len) continue;

    const double theta = (arc - b.cum_arc_A) / b.r_eff;
    const double ct = std::cos(theta), st = std::sin(theta);

    Eigen::Vector3d p = b.O + b.r_eff * (-ct * b.n1 + st * b.e_in);
    point.x = p.x(); point.y = p.y(); point.z = p.z();

    Eigen::Vector3d tan_pos = (st * b.n1 + ct * b.e_in) * impl_->total_length_;
    impl_->last_pos_tangent_ = {tan_pos.x(), tan_pos.y(), tan_pos.z()};

    SampleOrientation(arc, impl_->ori_arcs_, impl_->quats_, impl_->total_length_,
                      point, impl_->last_ang_tangent_);
    return;
  }

  // ── 直线段插值 ───────────────────────────────────────────────────────────
  int seg = impl_->n_ - 2;
  for (int i = 0; i < impl_->n_ - 1; ++i) {
    if (arc <= impl_->straight_cum_end_[i]) { seg = i; break; }
  }

  const double seg_span = impl_->straight_cum_end_[seg] - impl_->straight_cum_start_[seg];
  Eigen::Vector3d p;
  if (seg_span > 1e-12) {
    const double t = (arc - impl_->straight_cum_start_[seg]) / seg_span;
    const double d_start = (seg > 0)            ? impl_->blends_[seg - 1].d_cut : 0.0;
    const double d_end   = (seg < impl_->n_ - 2) ? impl_->blends_[seg].d_cut     : 0.0;
    const double len     = impl_->seg_lengths_[seg] - d_start - d_end;
    p = impl_->straight_start_pos_[seg] + (t * len) * impl_->seg_dirs_[seg];
  } else {
    p = impl_->straight_start_pos_[seg];
  }
  point.x = p.x(); point.y = p.y(); point.z = p.z();

  Eigen::Vector3d tan_pos = impl_->seg_dirs_[seg] * impl_->total_length_;
  impl_->last_pos_tangent_ = {tan_pos.x(), tan_pos.y(), tan_pos.z()};

  SampleOrientation(arc, impl_->ori_arcs_, impl_->quats_, impl_->total_length_,
                    point, impl_->last_ang_tangent_);
}

void CartesianMultiPointPath::EvaluateTangent(ArmMotion::Vector3d& pos,
                                              ArmMotion::Vector3d& omega) const {
  pos   = impl_->last_pos_tangent_;
  omega = impl_->last_ang_tangent_;
}

