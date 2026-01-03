#include "trajectory_cubic_planner.h"
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
    segment_.resize(N - 1); 
    // N-1 段三次多项式
    for (int i = 0; i < N - 1; i++) {
        double T = time_[i+1] - time_[i];
        segment_[i].a0 = theta_[i];
        segment_[i].a1 = vel_[i];
        segment_[i].a2 = 3 * (theta_[i+1] - theta_[i]) / (T * T) - 
                         (2 * vel_[i] + vel_[i+1]) / T;
        segment_[i].a3 = -2 * (theta_[i+1] - theta_[i]) / (T * T * T) + 
                         (vel_[i] + vel_[i+1]) / (T * T);
    }
    return true;
}

bool CubicTrajectoryPlanner::Plan(const std::vector<double>& theta, 
                                  const std::vector<double>& time)
{
    if (theta.size() < 2 || theta.size() != time.size()) {
        return false;
    }

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

double CubicTrajectoryPlanner::GetPosition(double t) const
{
    if (theta_.empty()) return 0.0;
    if (t <= time_.front()) return theta_.front();
    if (t >= time_.back()) return theta_.back();

    const int N = theta_.size();
    int k = 0;
    // 找到时间位于哪一段
    for (int i = 0; i < N-1; i++) {
        if (t >= time_[i] && t <= time_[i+1]) {
            double T = t - time_[i];
            double a0 = segment_[i].a0;
            double a1 = segment_[i].a1;
            double a2 = segment_[i].a2;
            double a3 = segment_[i].a3;
            return a0 + a1 * T + a2 * (T * T) + a3 * (T * T * T);
        }
    }
    return -1.0;
}

double CubicTrajectoryPlanner::GetVelocity(double t) const
{
    if (theta_.empty()) return 0.0;
    if (t <= time_.front()) return theta_.front();
    if (t >= time_.back()) return theta_.back();

    const int N = theta_.size();
    int k = 0;
    // 找到时间位于哪一段
    for (int i = 0; i < N-1; i++) {
        if (t >= time_[i] && t <= time_[i+1]) {
            double T = t - time_[i];
            double a1 = segment_[i].a1;
            double a2 = segment_[i].a2;
            double a3 = segment_[i].a3;
            return a1 + 2.0 * a2 * T + 3.0 * a3 * (T * T);
        }
    }
    return -1.0;
}

double CubicTrajectoryPlanner::GetAcceleration(double t) const
{
    if (theta_.empty()) return 0.0;
    if (t <= time_.front()) return theta_.front();
    if (t >= time_.back()) return theta_.back();

    const int N = theta_.size();
    int k = 0;
    // 找到时间位于哪一段
    for (int i = 0; i < N-1; i++) {
        if (t >= time_[i] && t <= time_[i+1]) {
            double T = t - time_[i];
            double a2 = segment_[i].a2;
            double a3 = segment_[i].a3;
            return 2.0 * a2 + 6.0 * a3 * T;
        }
    }
    return -1.0;
}
