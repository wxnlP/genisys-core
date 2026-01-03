#include <iostream>
#include <fstream>
#include <vector>
#include "trajectory_cubic_planner.h"
#include "trajectory_linear_planner.h"

int main()
{
    CubicTrajectoryPlanner planner;
    // LinearTrajectoryPlanner planner;

    // 用户指定的 via points
    std::vector<double> theta = {0.0,  0.8,  0.3,  1.2,  0.6,  1.5,
                                 1.0,  1.8,  1.2,  2.0,  1.6,  2.2};
    std::vector<double> time  = {0.0,  0.5,  1.5,  2.0,  3.0,  4.0,
                                 5.0,  5.5,  6.5,  7.0,  8.0,  9.0};

    // planner.SetAccel(2.0);
    if (!planner.Plan(theta, time)) {
        std::cerr << "Plan failed\n";
        return -1;
    }

    // 1. 导出密集轨迹
    std::ofstream traj_ofs("trajectory.csv");
    traj_ofs << "t,pos,vel,acc\n";
    for (double t = time.front(); t <= time.back(); t += 0.002) {
        TrajectoryPlannerBase::TrajectoryPoint_t s;
        planner.Sample(t, s);
        traj_ofs << t << "," << s.pos << "," << s.vel << "," << s.acc << "\n";
    }
    traj_ofs.close();

    // 2. 导出 via points（关键！）
    std::ofstream via_ofs("via_points.csv");
    via_ofs << "t_via,theta_via\n";
    for (size_t i = 0; i < theta.size(); ++i) {
        via_ofs << time[i] << "," << theta[i] << "\n";
    }
    via_ofs.close();

    std::cout << "trajectory.csv and via_points.csv generated\n";
    return 0;
}