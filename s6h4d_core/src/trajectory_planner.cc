#include "trajectory_planner.h"
#include <cassert>


/**
 * @brief Thomas 算法解三对角矩阵
*/
void CubicTrajectoryPlanner::SolveTridiagonal(const std::vector<double>& a, 
                                            const std::vector<double>& b,
                                            const std::vector<double>& c, 
                                            const std::vector<double>& d,
                                            std::vector<double>& x)
{
    int n = b.size();
    assert(a.size() == n);
    assert(c.size() == n);
    assert(d.size() == n);
    x.resize(n, 0.0);

    std::vector<double> c_prime(n, 0.0);
    std::vector<double> d_prime(n, 0.0);

    c_prime[0] = c[0] / b[0];
    d_prime[0] = d[0] / b[0];

    // 处理c_prime
    for (int i = 1; i < (n - 1); i++) {
        c_prime[i] = c[i] / (b[i] - a[i] * c_prime[i-1]);
    }
    // 处理d_prime
    for (int i = 1; i < n; i++) {
        d_prime[i] = (d[i] - a[i] * d_prime[i-1]) / (b[i] - a[i] * c_prime[i-1]);
    }
    // 返回结果
    x[n-1] = d_prime[n-1];
    for (int i = (n - 2); i >= 0; i--) {
        x[i] = d_prime[i] - c_prime[i] * x[i+1];
    }
}

bool CubicTrajectoryPlanner::CalculateViaPointVel()
{
    const int N = theta_.size();
    vel_.resize(N, 0.0);
    if (N < 2) { return false; }
    const int VIA_POINTS = N - 2;

    std::vector<double> a(VIA_POINTS, 0.0);
    std::vector<double> b(VIA_POINTS, 0.0);
    std::vector<double> c(VIA_POINTS, 0.0);
    std::vector<double> d(VIA_POINTS, 0.0);

    // 处理VIA_POINTS 个中间点速度
    for (int i = 0; i < VIA_POINTS; i++) {
        double t_prev = time_[i+1] - time_[i];
        double t_next = time_[i+2] - time_[i+1];
        if (i == 0) {
            a[i] = 0.0;
        } else {
            a[i] = t_next;
        }
        b[i] = 2 * (t_prev + t_next);
        if (i == VIA_POINTS - 1) {
            c[i] = 0.0;
        } else {
            c[i] = t_prev;
        }
        d[i] = 3 * ((theta_[i+2] - theta_[i+1]) / t_next + 
                    (theta_[i+1] - theta_[i]) / t_prev);
    }

    std::vector<double> x;
    SolveTridiagonal(a, b, c, d, x);

    vel_.resize(N, 0.0);
    vel_[0] = 0.0;
    vel_[N-1] = 0.0;
    for (int j = 0; j < VIA_POINTS; j++) {
        vel_[j+1] = x[j];
    }

    return true;
}


bool CubicTrajectoryPlanner::GenerateSegment()
{
    const int N = theta_.size();
    // 清理容器
    segment_.clear();
    // N-1 段三次多项式
    for (int i = 0; i < N - 1; i++) {
        double T = time_[i+1] - time_[i];
        double a0 = theta_[i];
        double a1 = vel_[i];
        double a2 = 3 * (theta_[i+1] - theta_[i]) / (T * T) - 
                    (2 * vel_[i] + vel_[i+1]) / T;
        double a3 = -2 * (theta_[i+1] - theta_[i]) / (T * T * T) + 
                    (vel_[i] + vel_[i+1]) / (T * T);
        segment_.emplace_back(a0, a1, a2, a3, time_[i], time_[i+1]);
    }
    return true;
}

bool CubicTrajectoryPlanner::Plan(const std::vector<double>& theta, 
                                  const std::vector<double>& time)
{
    assert(theta.size() == time.size());
    assert(theta.size() >= 2);
    theta_ = theta;
    time_ = time;
    if(!CalculateViaPointVel()) {
        return false;
    }
    if (!GenerateSegment()) {
        return false;
    }
    return true;
}

bool CubicTrajectoryPlanner::Sample(double t, CubicSegPoint_t& point)
{
    if (t <= time_.front()) {
        point.pos = theta_.front();
        point.acc = 0.0;
        point.vel = 0.0;
    }
    if (t >= time_.back()) {
        point.pos = theta_.back();
        point.acc = 0.0;
        point.vel = 0.0;
    }
    for (const auto& seg : segment_) {
        if (seg.InTimeSegment(t)) {
            point.pos = seg.GetPosition(t);
            point.acc = seg.GetAcceleration(t);
            point.vel = seg.GetVelocity(t);
            return true;
        }
    }
    return false;
}