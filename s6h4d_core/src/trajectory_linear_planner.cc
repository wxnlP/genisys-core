#include "trajectory_linear_planner.h"

void LinearTrajectoryPlanner::SetAccel(double a)
{
    accel_ = std::abs(a);
}

bool LinearTrajectoryPlanner::Plan(const std::vector<double>& theta, 
                                   const std::vector<double>& time)
{
    if (theta.size() < 2 || theta.size() != time.size()) {
        return false;
    }
    theta_ = theta;
    time_ = time;

    const int N = theta_.size();

    vel_.resize(N-1);
    tb_.resize(N);

    tb_[0] = std::abs(vel_[0]) / accel_;
    tb_[N-1] = std::abs(vel_[N-2]) / accel_;

    for (int i = 0; i < N-1; i++) {
        vel_[i] = (theta_[i+1] - theta_[i]) / (time_[i+1] - time_[i]);
        // 处理首端和尾部过渡段
        if (i >= 1 && i <= N-2) {
            tb_[i] = (vel_[i+1] - vel_[i]) / accel_;
        }
    }
    return true;
}

double LinearTrajectoryPlanner::GetPosition(double t) const
{
    if (theta_.empty()) return 0.0;
    if (t <= time_.front()) return theta_.front();
    if (t >= time_.back()) return theta_.back();

    const int N = theta_.size();
    int k = 0;
    // 找到时间位于哪一段
    for (int i = 0; i < N-1; i++) {
        if (t >= time_[i] && t <= time_[i+1]) {
            k = i;
            break;
        }
    }

    // 是否位于左过渡区
    if (k > 0) {
        double t_blend_start = time_[k] - 0.5 * tb_[k];
        double t_blend_end   = time_[k] + 0.5 * tb_[k];
        if (t >= t_blend_start && t <= t_blend_end) {
            double T = t - time_[k] + 0.5 * tb_[k];
            double T_prev = t - time_[k-1] - T;
            double a = (vel_[k] - vel_[k-1]) / tb_[k];
            return theta_[k-1] + vel_[k-1] * (T_prev + T) + 0.5 * a * (T * T);
        }
    }
    // 是否位于右过渡区
    if (k < N-1) {
        double t_blend_start = time_[k+1] - 0.5 * tb_[k+1];
        double t_blend_end   = time_[k+1] + 0.5 * tb_[k+1];
        if (t >= t_blend_start && t <= t_blend_end) {
            double T = t - time_[k+1] + 0.5 * tb_[k+1];
            double T_prev = t - time_[k] - T;
            double a;
            // 终止点
            if (k == N-2) {
                a = -vel_[k] / tb_[k+1];
            } else {
                a = (vel_[k+1] - vel_[k]) / tb_[k+1];
            }
            return theta_[k] + vel_[k] * (T_prev + T) + 0.5 * a * (T * T);
        }
    }
    return theta_[k] + vel_[k] * (t - time_[k]);
}

double LinearTrajectoryPlanner::GetVelocity(double t) const
{
    if (theta_.empty()) return 0.0;
    if (t <= time_.front()) return theta_.front();
    if (t >= time_.back()) return theta_.back();

    const int N = theta_.size();
    int k = 0;
    // 找到时间位于哪一段
    for (int i = 0; i < N-1; i++) {
        if (t >= time_[i] && t <= time_[i+1]) {
            k = i;
            break;
        }
    }
    // 是否位于左过渡区
    if (k > 0) {
        double t_blend_start = time_[k] - 0.5 * tb_[k];
        double t_blend_end   = time_[k] + 0.5 * tb_[k];
        if (t >= t_blend_start && t <= t_blend_end) {
            double T = t - time_[k] + 0.5 * tb_[k];
            double a = (vel_[k] - vel_[k-1]) / tb_[k];
            return vel_[k-1] +  a * T;
        }
    }
    // 是否位于右过渡区
    if (k < N-1) {
        double t_blend_start = time_[k+1] - 0.5 * tb_[k+1];
        double t_blend_end   = time_[k+1] + 0.5 * tb_[k+1];
        if (t >= t_blend_start && t <= t_blend_end) {
            double T = t - time_[k+1] + 0.5 * tb_[k+1];
            double a;
            // 终止点
            if (k == N-2) {
                a = -vel_[k] / tb_[k+1];
            } else {
                a = (vel_[k+1] - vel_[k]) / tb_[k+1];
            }
            return vel_[k]+ a * T;
        }
    }
    return vel_[k];
}

double LinearTrajectoryPlanner::GetAcceleration(double t) const
{
    if (theta_.empty()) return 0.0;
    if (t <= time_.front()) return theta_.front();
    if (t >= time_.back()) return theta_.back();

    const int N = theta_.size();
    int k = 0;
    // 找到时间位于哪一段
    for (int i = 0; i < N-1; i++) {
        if (t >= time_[i] && t <= time_[i+1]) {
            k = i;
            break;
        }
    }

    // 是否位于左过渡区
    if (k > 0) {
        double t_blend_start = time_[k] - 0.5 * tb_[k];
        double t_blend_end   = time_[k] + 0.5 * tb_[k];
        if (t >= t_blend_start && t <= t_blend_end) {
            return  (vel_[k] - vel_[k-1]) / tb_[k];
        }
    }
    // 是否位于右过渡区
    if (k < N-1) {
        double t_blend_start = time_[k+1] - 0.5 * tb_[k+1];
        double t_blend_end   = time_[k+1] + 0.5 * tb_[k+1];
        if (t >= t_blend_start && t <= t_blend_end) {
            // 终止点
            if (k == N-2) {
                return -vel_[k] / tb_[k+1];
            } else {
                return (vel_[k+1] - vel_[k]) / tb_[k+1];
            }
        }
    }
    return 0.0;
}