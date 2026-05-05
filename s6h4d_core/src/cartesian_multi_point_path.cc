#include "cartesian_multi_point_path.h"

#include <algorithm>
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// 构造 & 预计算
// ─────────────────────────────────────────────────────────────────────────────

CartesianMultiPointPath::CartesianMultiPointPath(
		std::vector<ArmMotion::Pose6Dof_t> points, double r)
		: points_(std::move(points)), r_(r) {
	Precompute();
}

void CartesianMultiPointPath::Precompute() {
	n_ = static_cast<int>(points_.size());
	if (n_ < 2) {
		total_length_ = 0.0;
		return;
	}

	// ── Step 1: 提取位置和四元数 ─────────────────────────────────────────────
	pos_.resize(n_);
	quats_.resize(n_);
	for (int i = 0; i < n_; ++i) {
		pos_[i] = {points_[i].x, points_[i].y, points_[i].z};
		quats_[i] = Eigen::Quaterniond(points_[i].quat.w, points_[i].quat.x,
																	 points_[i].quat.y, points_[i].quat.z).normalized();
	}
	// 保证四元数沿最短路径插值
	for (int i = 1; i < n_; ++i) {
		if (quats_[i - 1].dot(quats_[i]) < 0.0) quats_[i].coeffs() *= -1.0;
	}

	// ── Step 2: 各线段方向与长度 ─────────────────────────────────────────────
	seg_dirs_.resize(n_ - 1);
	seg_lengths_.resize(n_ - 1);
	for (int i = 0; i < n_ - 1; ++i) {
		Eigen::Vector3d delta = pos_[i + 1] - pos_[i];
		seg_lengths_[i] = delta.norm();
    if (seg_lengths_[i] > 1e-12) { 
      seg_dirs_[i] = delta / seg_lengths_[i]; 
    } else {
      seg_dirs_[i] = Eigen::Vector3d::UnitX(); 
    }
	}

	// ── Step 3: 各中间路径点的圆弧融合参数 ───────────────────────────────────
	// blends_[j-1] 对应路径点 j（j = 1 .. n-2）
	blends_.resize(n_ - 2);
	for (int j = 1; j <= n_ - 2; ++j) {
		BlendArc& b = blends_[j - 1];
		const Eigen::Vector3d& e_in  = seg_dirs_[j - 1];
		const Eigen::Vector3d& e_out = seg_dirs_[j];

		const double dot  = std::max(-1.0, std::min(1.0, e_in.dot(e_out)));
		const double beta = std::acos(dot);

		// 几乎共线，不需要融合
		if (beta < 1e-6) { b = {}; continue; }

		// 切点距离 d = r / tan(beta/2)，裁剪到相邻线段的一半
		double d = r_ / std::tan(beta * 0.5);
		d = std::min({d, seg_lengths_[j - 1] * 0.499, seg_lengths_[j] * 0.499});
		const double r_eff = d * std::tan(beta * 0.5);

		// 垂直于 e_in 的内法向量（指向圆心一侧）
		Eigen::Vector3d n1 = e_out - dot * e_in;
		if (n1.norm() < 1e-12) { b = {}; continue; }
		n1.normalize();

		// 圆弧起点 A = P[j] - d*e_in，圆心 O = A + r_eff*n1
		b.r_eff   = r_eff;
		b.beta    = beta;
		b.d_cut   = d;
		b.arc_len = r_eff * beta;
		b.e_in    = e_in;
		b.n1      = n1;
		b.O       = pos_[j] - d * e_in + r_eff * n1;
		b.cum_arc_A = 0.0;  // 在 Step 4 填入
	}

	// ── Step 4: 累计弧长 ──────────────────────────────────────────────────────
	// 结构：[直线 0] [弧 1] [直线 1] [弧 2] ... [直线 n-2]
	straight_cum_start_.resize(n_ - 1);
	straight_cum_end_.resize(n_ - 1);
	straight_start_pos_.resize(n_ - 1);

	double arc = 0.0;
	for (int i = 0; i < n_ - 1; ++i) {
		// 直线段 i 被前端切点 (B[i]，即上一弧的出口) 和后端切点 (A[i+1]) 截短
		const double d_start = (i > 0)      ? blends_[i - 1].d_cut : 0.0;
		const double d_end   = (i < n_ - 2) ? blends_[i].d_cut     : 0.0;

		// 直线段实际起点位置
		straight_start_pos_[i] = pos_[i] + d_start * seg_dirs_[i];

		straight_cum_start_[i] = arc;
		arc += seg_lengths_[i] - d_start - d_end;  // 保证 >= 0（裁剪已保证）
		straight_cum_end_[i] = arc;

		// 直线段之后紧接圆弧（末段之后无圆弧）
		if (i < n_ - 2) {
			blends_[i].cum_arc_A = arc;
			arc += blends_[i].arc_len;
		}
	}
	total_length_ = arc;

	// ── Step 5: 姿态插值路标弧长 ─────────────────────────────────────────────
	// 端点在 0 / total_length_，中间路径点取其圆弧弧长中点
	ori_arcs_.resize(n_);
	ori_arcs_[0]       = 0.0;
	ori_arcs_[n_ - 1]  = total_length_;
	for (int j = 1; j <= n_ - 2; ++j) {
		const BlendArc& b = blends_[j - 1];
		ori_arcs_[j] = (b.arc_len > 0.0)
											 ? b.cum_arc_A + b.arc_len * 0.5
											 : straight_cum_end_[j - 1];  // 无圆弧时取线段末端
	}
}

// ─────────────────────────────────────────────────────────────────────────────
// 姿态插值辅助（内部）
// ─────────────────────────────────────────────────────────────────────────────

// 根据当前弧长 arc，对 [q[k], q[k+1]] 做 Slerp，并更新 angular 切向量缓存。
static void SampleOrientation(
		double arc,
		const std::vector<double>& ori_arcs,
		const std::vector<Eigen::Quaterniond>& quats,
		double total_length,
		ArmMotion::Pose6Dof_t& out_point,
		ArmMotion::Vector3d& out_ang_tangent) {
	const int n = static_cast<int>(ori_arcs.size());

	// 找到姿态所在区间 k
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

	// 角速度切向量 = axis*angle(Δq) * total_length / arc_span
	// 与 CartesianLinearPath 保持相同约定（body-frame rotation vector）
	Eigen::Quaterniond dq = quats[k].inverse() * quats[k + 1];
	if (dq.w() < 0.0) dq.coeffs() *= -1.0;  // 保证最短路径
	Eigen::AngleAxisd aa(dq);
	const double scale = (span > 1e-12) ? (total_length / span) : 0.0;
	Eigen::Vector3d omega = aa.axis() * aa.angle() * scale;
	out_ang_tangent = {omega.x(), omega.y(), omega.z()};
}

// ─────────────────────────────────────────────────────────────────────────────
// 路径采样
// ─────────────────────────────────────────────────────────────────────────────

// 圆弧公式（以 A[j] 为起点，θ ∈ [0, beta]，弧长参数 = r_eff * θ）：
//
//   R_x = -n1  （从 O 指向 A 的单位向量）
//   R_y = e_in （arc 切线方向，θ=0 时切线 = e_in）
//
//   p(θ) = O + r * (-cos(θ)*n1 + sin(θ)*e_in)
//
//   dp/dθ = r * (sin(θ)*n1 + cos(θ)*e_in)
//   dp/ds = (sin(θ)*n1 + cos(θ)*e_in) * total_length   （单位切向量 × L_total）

void CartesianMultiPointPath::EvaluatePos(double s,
																					 ArmMotion::Pose6Dof_t& point) const {
	if (n_ < 2) return;

	s = std::max(0.0, std::min(1.0, s));
	const double arc = s * total_length_;

	// ── 检查圆弧融合区 ───────────────────────────────────────────────────────
	for (int j = 1; j <= n_ - 2; ++j) {
		const BlendArc& b = blends_[j - 1];
		if (b.arc_len <= 0.0) continue;
		if (arc < b.cum_arc_A) break;  // 还未到达（blends_ 按顺序排列）
		if (arc > b.cum_arc_A + b.arc_len) continue;

		const double theta = (arc - b.cum_arc_A) / b.r_eff;
		const double ct = std::cos(theta), st = std::sin(theta);

		// 位置：p(θ) = O + r*(-cos(θ)*n1 + sin(θ)*e_in)
		Eigen::Vector3d p = b.O + b.r_eff * (-ct * b.n1 + st * b.e_in);
		point.x = p.x(); point.y = p.y(); point.z = p.z();

		// 位置切向量：dp/ds = (sin(θ)*n1 + cos(θ)*e_in) * total_length
		Eigen::Vector3d tan_pos = (st * b.n1 + ct * b.e_in) * total_length_;
		last_pos_tangent_ = {tan_pos.x(), tan_pos.y(), tan_pos.z()};

		// 姿态
		SampleOrientation(arc, ori_arcs_, quats_, total_length_,
											point, last_ang_tangent_);
		return;
	}

	// ── 直线段插值 ───────────────────────────────────────────────────────────
	int seg = n_ - 2;
	for (int i = 0; i < n_ - 1; ++i) {
		if (arc <= straight_cum_end_[i]) { seg = i; break; }
	}

	const double seg_span = straight_cum_end_[seg] - straight_cum_start_[seg];
	Eigen::Vector3d p;
	if (seg_span > 1e-12) {
		const double t = (arc - straight_cum_start_[seg]) / seg_span;
		// 直线段的实际长度（已去除两端切点截断量）
		const double d_start = (seg > 0)      ? blends_[seg - 1].d_cut : 0.0;
		const double d_end   = (seg < n_ - 2) ? blends_[seg].d_cut     : 0.0;
		const double len     = seg_lengths_[seg] - d_start - d_end;
		p = straight_start_pos_[seg] + (t * len) * seg_dirs_[seg];
	} else {
		p = straight_start_pos_[seg];
	}
	point.x = p.x(); point.y = p.y(); point.z = p.z();

	// 位置切向量
	Eigen::Vector3d tan_pos = seg_dirs_[seg] * total_length_;
	last_pos_tangent_ = {tan_pos.x(), tan_pos.y(), tan_pos.z()};

	// 姿态
	SampleOrientation(arc, ori_arcs_, quats_, total_length_,
										point, last_ang_tangent_);
}

void CartesianMultiPointPath::EvaluateTangent(ArmMotion::Vector3d& pos,
																							 ArmMotion::Vector3d& omega) const {
	pos   = last_pos_tangent_;
	omega = last_ang_tangent_;
}

