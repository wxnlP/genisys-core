#include "cartesian_linear_path.h"
#include <Eigen/Dense>

struct CartesianLinearPath::Impl {
    Eigen::Vector3d p_start_, p_target_;
    Eigen::Quaterniond q_start_, q_target_;
};

CartesianLinearPath::CartesianLinearPath(const ArmMotion::Pose6Dof& pose_start, const ArmMotion::Pose6Dof& pose_target)
    : impl_(std::make_unique<Impl>())
{
    // 1. 初始化位置矢量
    impl_->p_start_ = { pose_start.x, pose_start.y, pose_start.z };
    impl_->p_target_ = { pose_target.x, pose_target.y, pose_target.z };

    // 2. 初始化四元数
    impl_->q_start_ = Eigen::Quaterniond(pose_start.quat.w, pose_start.quat.x, pose_start.quat.y, pose_start.quat.z);
    impl_->q_target_ = Eigen::Quaterniond(pose_target.quat.w, pose_target.quat.x, pose_target.quat.y, pose_target.quat.z);

    // 3. 处理最短路径（防止 Slerp 绕远路）
    if (impl_->q_start_.dot(impl_->q_target_) < 0.0) {
        impl_->q_target_.coeffs() *= -1.0;
    }
    
}

CartesianLinearPath::~CartesianLinearPath() = default;

void CartesianLinearPath::EvaluatePos(double s, ArmMotion::Pose6Dof& path) const
{
    // 1. 线性插值位置
    Eigen::Vector3d p_s = impl_->p_start_ + s * (impl_->p_target_ - impl_->p_start_);

    // 2. 球面线性插值姿态
    Eigen::Quaterniond q_s = impl_->q_start_.slerp(s, impl_->q_target_);

    // 3. 将结果存入 path 向量 (x, y, z, qw, qx, qy, qz)
    path.x = p_s.x();
    path.y = p_s.y();
    path.z = p_s.z();
    path.quat.w = q_s.w();
    path.quat.x = q_s.x();
    path.quat.y = q_s.y();
    path.quat.z = q_s.z();
}

void CartesianLinearPath::EvaluateTangent(ArmMotion::Vector3d& pos, ArmMotion::Vector3d& omega) const
{
    // 1. 位置切线（常数）
    Eigen::Vector3d dp_ds = impl_->p_target_ - impl_->p_start_;

    // 2. 姿态切线（几何切线）
    // 计算相对旋转
    Eigen::Quaterniond q_rel = impl_->q_start_.inverse() * impl_->q_target_;
    // 将相对旋转转换为角轴表示
    Eigen::AngleAxisd aa(q_rel);
    // 计算旋转切线（轴角）
    Eigen::Vector3d omega_s = aa.axis() * aa.angle();

    pos.x = dp_ds.x();
    pos.y = dp_ds.y();
    pos.z = dp_ds.z();
    omega.x = omega_s.x();
    omega.y = omega_s.y();
    omega.z = omega_s.z();
}
