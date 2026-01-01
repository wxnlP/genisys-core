#include <iostream>
#include <fstream>
#include <vector>
#include "trajectory_planner.h"

int main()
{
    CubicTrajectoryPlanner planner;

    std::vector<double> theta = {0.0,  0.8,  0.3,  1.2,  0.6,  1.5,
                                 1.0,  1.8,  1.2,  2.0,  1.6,  2.2};
    std::vector<double> time  = {0.0,  0.5,  1.5,  2.0,  3.0,  4.0,
                                 5.0,  5.5,  6.5,  7.0,  8.0,  9.0};

    if (!planner.Plan(theta, time)) {
        std::cerr << "Plan failed\n";
        return -1;
    }
    std::ofstream ofs("trajectory.csv");
    ofs << "t,pos,vel,acc\n";

    for (double t = 0.0; t <= 3.0; t += 0.002) {
        CubicTrajectoryPlanner::CubicSegPoint_t s;
        planner.Sample(t, s);
        ofs << t << "," << s.pos << "," << s.vel << "," << s.acc << "\n";
    }

    ofs.close();
    std::cout << "trajectory.csv generated\n";
    return 0;
}