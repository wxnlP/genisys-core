#include "kinematic.h"

#include <Eigen/Dense>
#include <cmath>
#include <cstring>
#include <iostream>
#include <iomanip>

int main() {
  // ArmMotion::MDHParams mdh_params = {
  //     .alpha = {0.0, -M_PI_2, 0.0, 0.0, -M_PI_2, M_PI_2},        // alpha
  //     .a = {0.0, 0.02, 0.26, 0.24668, 0.075, 0.0},               // a
  //     .d = {0.1207, 0.0, 0.0, 0.0, 0.0, 0.0265},                 // d
  //     .theta = {0.0, -M_PI, M_PI - 0.233176, 0.233176, M_PI_2, 0.0}  // theta (初始偏移量)
  // };
  ArmMotion::MDHParams mdh_params = {
    .alpha = {0.0, -M_PI_2, 0.0, -M_PI_2, M_PI_2, -M_PI_2},  // alpha
    .a = {0.0, 0.04, 0.2, 0.059, 0.0, 0.0},             // a
    .d = {0.123, 0.0, 0.0, 0.2, 0.0, 0.104},       // d
    .theta = {0.0, -M_PI_2, 0.0, 0.0, 0.0, 0.0}          // theta (初始偏移量)
  };
  KinematicSolver kSolver(mdh_params);
  // const ArmMotion::MDHConfig_t mdh_config{0.123, 0.04, 0.20, 0.059, 0.2, 0.104};
  // KinematicSolver kSolver(mdh_config);
  // kSolver.SetIKMode(KinematicSolver::IKMode::ParallelWrist);
  ArmMotion::Joint6Dof_t joint{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ArmMotion::Pose6Dof_t pose;
  kSolver.FK(joint, pose);

  std::cout << pose.x << " " << pose.y << " " << pose.z << " " << std::endl;
  std::cout << pose.quat.w << " " << pose.quat.x << " " << pose.quat.y << " " <<  pose.quat.z << " " << std::endl;

   ArmMotion::IKSolves_t outputSolves;
  kSolver.IK(pose, joint, outputSolves);
  for (int i = 0; i < 8; ++i) {
    bool valid = false;

    std::cout << "解 " << i + 1 << ": ";

    for (int j = 0; j < 6; ++j) {
      double deg = outputSolves.solves[i].theta[j] *( 180.0 / M_PI);
      std::cout << std::setw(9) << deg << " ";
    }
    std::cout << " | flag=[" << static_cast<int>(outputSolves.solveFlag[i][0]) << ","
         << static_cast<int>(outputSolves.solveFlag[i][1]) << ","
         << static_cast<int>(outputSolves.solveFlag[i][2]) << "]";

    if (valid) {
      std::cout << " | score=";
    } else {
      std::cout << " | score=invalid";
    }
    std::cout << std::endl;
  }

}