#pragma once

#include <vector>
#include <Eigen/Dense>

class QuaternionTrajectoryPlanner
{
public:
    bool Plan(const std::vector<Eigen::Quaterniond>& q,
              const std::vector<double>& time) {
        if (q.size() < 2 || q.size() != time.size()) {
            return false;
        }

        q_.clear();
        q_.reserve(q.size());

        time_ = time;

        q_.push_back(q[0].normalized());
        for (size_t i = 1; i < q.size(); ++i) {
            Eigen::Quaterniond current_q = q[i].normalized();
            // 如果点积为负，说明走的是长弧，需要翻转符号
            if (q_.back().dot(current_q) < 0.0) {
                current_q.coeffs() *= -1.0;
            }
            q_.push_back(current_q);
        }

        return true;
    }

    bool Sample(double t, Eigen::Quaterniond& q) const {
        if (q_.empty()) {
            return false;
        }

        if (t <= time_.front()) {
            q = q_.front();
            return true;
        }

        if (t >= time_.back()) {
            q = q_.back();
            return true;
        }

        int index = 0;
        for (int i = 0; i < time_.size()-1; i++) {
            if (t >= time_[i] && t <= time_[i+1]) {
                index = i;
                break;
            }
        }

        double t0 = time_[index];
        double t1 = time_[index+1];
        double alpha = (t - t0) / (t1 - t0);
        q = q_[index].slerp(alpha, q_[index+1]);
        q.normalize();
        return true;
    }
private:
    std::vector<Eigen::Quaterniond> q_;
    std::vector<double> time_;
};