#include "trajectory_linear_planner.h"
#include <cmath>


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
    accel_.resize(N);

    // 检查时间间隔是否足够
    for (int i = 0; i < N-1; i++) {
        double dt = time_[i+1] - time_[i];
        if (dt < tb_) {
            // 时间间隔太小，无法完成过渡
            // 可以调整tb_或报错
            return false;
        }
    }

    // 速度计算
    vel_[0] = (theta_[1] - theta_[0]) / (time_[1] - time_[0] - 0.5 * tb_);
    vel_[N-2] = (theta_[N-1] - theta_[N-2]) / (time_[N-1] - time_[N-2] - 0.5 * tb_);
    for (int i = 1; i < N-2; i++) {
        vel_[i] = (theta_[i+1] - theta_[i]) / (time_[i+1] - time_[i]);
    }

    // 加速度计算
    accel_[0] = (vel_[0] - start_vel_) / (0.5 * tb_);
    accel_[N-1] = (end_vel_ - vel_[N-2]) / (0.5 * tb_);
    for (int i = 1; i < N-1; i++) {
        // 中间过渡：从上一段速度过渡到下一段速度
         accel_[i] = (vel_[i] - vel_[i-1]) / tb_;
    }

    for (int i = 1; i < N-1; ++i) {
        vel_[i] = (theta_[i+1] - theta_[i] - 0.5 * tb_ * vel_[i-1]) / (time_[i+1] - time_[i] - 0.5 * tb_);
    }

    // 过渡区位置计算
    pos_pre_blend_.resize(N);
    pos_post_blend_.resize(N-1);

    pos_pre_blend_[0] = theta_[0];

    double current_pos = theta_[0];
    double current_t = time_[0];

    // 处理第一个加速段
    double dt = 0.5 * tb_;
    double v = start_vel_;
    double a = accel_[0];
    current_pos += v * dt + 0.5 * a * dt * dt;
    current_t += dt;

    pos_post_blend_[0] = current_pos;

    for (int i = 0; i < N-2; ++i) {
        // 匀速段
        double t_cruise = time_[i+1] - time_[i] - tb_;
        if (t_cruise > 0) {
            current_pos += vel_[i] * t_cruise;
            current_t += t_cruise;
            pos_pre_blend_[i+1] = current_pos;
        }
        dt = tb_;
        v = vel_[i];
        a = accel_[i+1];
        current_pos += v * dt + 0.5 * a * dt * dt;
        current_t += dt;

        pos_post_blend_[i+1] = current_pos;
    }

    double t_cruise   = time_.back() - time_[N-2] - tb_;      
    double dt_blend   = 0.5 * tb_;

    double pos_start = pos_post_blend_[N-2];        
    double delta_pos = theta_.back() - pos_start;

    // 平均速度定理（可以略去加速度依赖计算速度）
    double v_coeff   = t_cruise + 0.5 * dt_blend;
    double constant  = 0.5 * end_vel_ * dt_blend;
    double v_new = (delta_pos - constant) / v_coeff;

    // 更新速度和加速度
    vel_[N-2] = v_new;
    accel_[N-1] = (end_vel_ - v_new) / dt_blend;

    // ==================== 计算最后一段的位置缓存 ====================
    current_pos = pos_start;

    // 计算匀速段
    if (t_cruise > 0.0) {
        current_pos += v_new * t_cruise;
        pos_pre_blend_.back() = current_pos; 
    }

    return true;
}

double LinearTrajectoryPlanner::GetPosition(double t) const
{
    if (theta_.empty()) { return 0.0; }
    if (t <= time_.front()) { return theta_.front(); }
    if (t >= time_.back()) { return theta_.back(); }

    const int N = theta_.size();

    int k  = -1;
    // 找到时间位于哪一段
    for (int i = 0; i < N-1; i++) {
        if (t >= time_[i] && t <= time_[i+1]) {
            k  = i;
            break;
        }
    }

    if (k  == -1) { return false; }
    // double t_blend_start1 = time_[k] - 0.5 * tb_;
    double t_blend_end1   = time_[k] + 0.5 * tb_;
    double t_blend_start2 = time_[k+1] - 0.5 * tb_;
    // double t_blend_end2   = time_[k+1] + 0.5 * tb_;

    double a;
    if (t <= t_blend_end1) {
        // 左过渡
        double dt = t -  time_[k] + 0.5 * tb_;
        a = accel_[k];
        if (k == 0) {
            dt = t -  time_[0];
            return theta_[0] + start_vel_ * dt + 0.5 * a * dt * dt;
        }
        return pos_pre_blend_[k] + vel_[k-1] * dt + 0.5 * a * dt * dt;
    } else if (t >= t_blend_start2) {
        // 右过渡
        double dt = t - time_[k+1] + 0.5 * tb_;
        a = accel_[k+1];
        return pos_pre_blend_[k+1] + vel_[k] * dt + 0.5 * a * dt * dt;
    } else {
        // 匀速
        double dt = t - time_[k] - 0.5 * tb_;
        return pos_post_blend_[k] + vel_[k] * dt; 
    }
}

double LinearTrajectoryPlanner::GetVelocity(double t) const
{
    if (theta_.empty()) { return 0.0; }
    if (t <= time_.front()) { return vel_.front(); }
    if (t >= time_.back()) { return vel_.back(); }

    const int N = theta_.size();
    int k = 0;
    // 找到时间位于哪一段
    for (int i = 0; i < N-1; i++) {
        if (t >= time_[i] && t <= time_[i+1]) {
            k = i;
            break;
        }
    }

    // double t_blend_start1 = time_[k] - 0.5 * tb_;
    double t_blend_end1   = time_[k] + 0.5 * tb_;
    double t_blend_start2 = time_[k+1] - 0.5 * tb_;
    // double t_blend_end2   = time_[k+1] + 0.5 * tb_;

    double a;
    if (t <= t_blend_end1) {
        // 左过渡
        double dt = t -  time_[k] + 0.5 * tb_;
        a = accel_[k];
        if (k == 0) {
            dt = t -  time_[0];
            return start_vel_ + a * dt;
        }
        return vel_[k-1] + a * dt;
    } else if (t >= t_blend_start2) {
        // 右过渡
        double dt = t - time_[k+1] + 0.5 * tb_;
        a = accel_[k+1];
        return vel_[k] + a * dt;
    } else {
        // 匀速
        return vel_[k]; 
    }
}

double LinearTrajectoryPlanner::GetAcceleration(double t) const
{
    if (theta_.empty()) { return 0.0; }
    if (t <= time_.front()) { return 0.0; }
    if (t >= time_.back()) { return 0.0; }

    const int N = theta_.size();
    int k = 0;
    // 找到时间位于哪一段
    for (int i = 0; i < N-1; i++) {
        if (t >= time_[i] && t <= time_[i+1]) {
            k = i;
            break;
        }
    }

    // double t_blend_start1 = time_[k] - 0.5 * tb_;
    double t_blend_end1   = time_[k] + 0.5 * tb_;
    double t_blend_start2 = time_[k+1] - 0.5 * tb_;
    // double t_blend_end2   = time_[k+1] + 0.5 * tb_;

    double a;
    if (t <= t_blend_end1) {
        // 左过渡
        a = accel_[k];
        return a;
    } else if (t >= t_blend_start2) {
        // 右过渡
        a = accel_[k+1];
        return a;
    } else {
        // 匀速
        return 0.0; 
    }
}
