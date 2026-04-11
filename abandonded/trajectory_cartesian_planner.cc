#include "trajectory_cartesian_planner.h"
#include <cmath>

bool CartesianTrajectoryPlanner::Plan(
    const std::vector<ArmMotion::Pose6Dof_t>& path,
    const std::vector<double>& time) 
{
    if (path.size() < 2 || path.size() != time.size()) {
        return false;
    }
    time_ = time;
    path_ = path;

    std::vector<double> p[3];
    std::vector<Eigen::Quaterniond> quat;

    for (const auto& pose : path_) {
        p[0].push_back(pose.x);
        p[1].push_back(pose.y);
        p[2].push_back(pose.z);

        Eigen::Quaterniond q(pose.quat.w, pose.quat.x, pose.quat.y, pose.quat.z);
        if (!quat.empty()) {
            if (quat.back().dot(q) < 0.0) {
                q.coeffs() = -q.coeffs(); // 确保选择最短路径
            }
        }
        quat.push_back(q);
    }

    // 位置轨迹规划
    for (int i = 0; i < 3; i++) {
        if (!linear_planner_[i].Plan(p[i], time)) {
            return false;
        }
    }
    // 姿态轨迹规划
    if (!quat_planner_.Plan(quat, time)) {
        return false;
    }
    
    return true;
}

bool CartesianTrajectoryPlanner::Sample(double t, CartesianPoint_t& point)
{
    TrajectoryPlannerBase::TrajectoryPoint_t p;
    Eigen::Quaterniond q;

    if (t <= time_.front()) {
        point.x.pos = path_.front().x;
        point.x.vel = 0.0;
        point.x.acc = 0.0;
        point.y.pos = path_.front().y;
        point.y.vel = 0.0;
        point.y.acc = 0.0;
        point.z.pos = path_.front().z;
        point.z.vel = 0.0;
        point.z.acc = 0.0;
        point.q.w = path_.front().quat.w;
        point.q.x = path_.front().quat.x;
        point.q.y = path_.front().quat.y;
        point.q.z = path_.front().quat.z;
        return true;
    }
    if (t >= time_.back()) {
        point.x.pos = path_.back().x;
        point.x.vel = 0.0;
        point.x.acc = 0.0;
        point.y.pos = path_.back().y;
        point.y.vel = 0.0;
        point.y.acc = 0.0;
        point.z.pos = path_.back().z;
        point.z.vel = 0.0;
        point.z.acc = 0.0;
        point.q.w = path_.back().quat.w;
        point.q.x = path_.back().quat.x;
        point.q.y = path_.back().quat.y;
        point.q.z = path_.back().quat.z;
        return true;
    }

    if (!linear_planner_[0].Sample(t, p)) {
        return false;
    }
    point.x.pos = p.pos;
    point.x.vel = p.vel;
    point.x.acc = p.acc;
    if (!linear_planner_[1].Sample(t, p)) {
        return false;
    }
    point.y.pos = p.pos;
    point.y.vel = p.vel;
    point.y.acc = p.acc;
    if (!linear_planner_[2].Sample(t, p)) {
        return false;
    }
    point.z.pos = p.pos;
    point.z.vel = p.vel;
    point.z.acc = p.acc;

    Eigen::Vector3d omega;
    if (!quat_planner_.Sample(t, q, omega)) {
        return false;
    }
    point.q.w = q.w();
    point.q.x = q.x();
    point.q.y = q.y();
    point.q.z = q.z();
    point.omega.x = omega[0];
    point.omega.y = omega[1];
    point.omega.z = omega[2];
    return true;
}
