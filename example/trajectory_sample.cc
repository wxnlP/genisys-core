#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "arm_logger.h"
#include "cartesian_linear_path.h"
#include "cartesian_trajectory.h"
#include "cubic_time_scaler.h"
#include "joint_path.h"
#include "joint_trajectory.h"
#include "lspb_time_scaler.h"

namespace {

void PrintJointPoint(const ArmMotion::JointTrajectoryPoint_t& point) {
  std::cout << "t=" << std::setw(4) << point.t << "  s=" << point.s << "  q=[";
  for (int i = 0; i < 6; ++i) {
    std::cout << point.position.q[i];
    if (i < 5) {
      std::cout << ", ";
    }
  }
  std::cout << "]  dq0=" << point.velocity.q[0]
            << "  ddq0=" << point.acceleration.q[0] << '\n';
}

void PrintCartesianPoint(const ArmMotion::CartesianTrajectoryPoint_t& point) {
  std::cout << "t=" << std::setw(4) << point.t << "  s=" << point.s << "  p=["
            << point.pose.x << ", " << point.pose.y << ", " << point.pose.z
            << "]"
            << "  v=[" << point.velocity.linear.x << ", "
            << point.velocity.linear.y << ", " << point.velocity.linear.z << "]"
            << "  w=[" << point.velocity.angular.x << ", "
            << point.velocity.angular.y << ", " << point.velocity.angular.z
            << "]" << '\n';
}

void WriteJointHeader(std::ofstream& stream) {
  stream << "t,q1,q2,q3,q4,q5,q6,dq1,dq2,dq3,dq4,dq5,dq6,ddq1,ddq2,ddq3,ddq4,"
            "ddq5,ddq6\n";
}

void WriteJointRow(std::ofstream& stream,
                   const ArmMotion::JointTrajectoryPoint_t& point) {
  stream << point.t;
  for (int i = 0; i < 6; ++i) {
    stream << ',' << point.position.q[i];
  }
  for (int i = 0; i < 6; ++i) {
    stream << ',' << point.velocity.q[i];
  }
  for (int i = 0; i < 6; ++i) {
    stream << ',' << point.acceleration.q[i];
  }
  stream << '\n';
}

void WriteCartesianHeader(std::ofstream& stream) {
  stream << "t,s,sd,sdd,x,y,z,qw,qx,qy,qz,vx,vy,vz,wx,wy,wz,ax,ay,az,alphax,"
            "alphay,alphaz\n";
}

void WriteCartesianRow(std::ofstream& stream,
                       const ArmMotion::CartesianTrajectoryPoint_t& point) {
  stream << point.t << ',' << point.s << ',' << point.sd << ',' << point.sdd
         << ',' << point.pose.x << ',' << point.pose.y << ',' << point.pose.z
         << ',' << point.pose.quat.w << ',' << point.pose.quat.x << ','
         << point.pose.quat.y << ',' << point.pose.quat.z << ','
         << point.velocity.linear.x << ',' << point.velocity.linear.y << ','
         << point.velocity.linear.z << ',' << point.velocity.angular.x << ','
         << point.velocity.angular.y << ',' << point.velocity.angular.z << ','
         << point.acceleration.linear.x << ',' << point.acceleration.linear.y
         << ',' << point.acceleration.linear.z << ','
         << point.acceleration.angular.x << ',' << point.acceleration.angular.y
         << ',' << point.acceleration.angular.z << '\n';
}

bool SampleJointPoint(const JointTrajectory& trajectory, double t,
                      ArmMotion::JointTrajectoryPoint_t& point) {
  if (trajectory.Sample(t, point)) {
    return true;
  }
  return t > 0.0 && trajectory.Sample(std::nextafter(t, 0.0), point);
}

bool SampleCartesianPoint(const CartesianTrajectory& trajectory, double t,
                          ArmMotion::CartesianTrajectoryPoint_t& point) {
  if (trajectory.Sample(t, point)) {
    return true;
  }
  return t > 0.0 && trajectory.Sample(std::nextafter(t, 0.0), point);
}

}  // namespace

int main() {
  std::cout << std::fixed << std::setprecision(4);

  ArmMotion::Joint6Dof_t joint_start(0.0, 0.1, 0.2, 0.0, -0.1, 0.3);
  ArmMotion::Joint6Dof_t joint_target(0.5, -0.2, 0.4, 0.3, 0.2, -0.1);
  JointPath joint_path(joint_start, joint_target);

  CubicTimeScaler joint_scaler;
  TimeScalerBase::PlanConfig joint_config;
  joint_config.T = 2.0;
  if (!joint_scaler.Plan(joint_config)) {
    ARM_LOG_ERROR("joint time scaler plan failed");
    return 1;
  }

  JointTrajectory joint_trajectory(joint_path, joint_scaler);

  ArmMotion::Pose6Dof_t pose_start;
  pose_start.x = 0.30;
  pose_start.y = -0.10;
  pose_start.z = 0.20;
  pose_start.quat.w = 1.0;

  ArmMotion::Pose6Dof_t pose_target;
  pose_target.x = 0.45;
  pose_target.y = 0.15;
  pose_target.z = 0.35;
  pose_target.quat.w = 0.9239;
  pose_target.quat.z = 0.3827;

  CartesianLinearPath cartesian_path(pose_start, pose_target);

  LSPBTimeScaler cartesian_scaler;
  TimeScalerBase::PlanConfig cartesian_config;
  cartesian_config.T = 2.0;
  cartesian_config.v_max = 0.75;
  cartesian_config.a_max = 1.0;
  if (!cartesian_scaler.Plan(cartesian_config)) {
    ARM_LOG_ERROR("cartesian time scaler plan failed");
    return 1;
  }

  CartesianTrajectory cartesian_trajectory(cartesian_path, cartesian_scaler);
  const double dt = 0.01;
  const double total_duration = joint_trajectory.GetDuration();
  std::ofstream joint_csv("joint_pos.csv");
  std::ofstream cartesian_csv("cartesian.csv");
  double last_sample_time = 0.0;

  if (!joint_csv.is_open() || !cartesian_csv.is_open()) {
    ARM_LOG_ERROR("failed to open csv output files");
    return 1;
  }

  joint_csv << std::fixed << std::setprecision(6);
  cartesian_csv << std::fixed << std::setprecision(6);
  WriteJointHeader(joint_csv);
  WriteCartesianHeader(cartesian_csv);

  for (double t = 0.0; t < total_duration; t += dt) {
    ArmMotion::JointTrajectoryPoint_t joint_point;
    ArmMotion::CartesianTrajectoryPoint_t cartesian_point;

    if (!SampleJointPoint(joint_trajectory, t, joint_point)) {
      ARM_LOG_ERROR("joint trajectory sample failed at t={}", t);
      return 1;
    }
    if (!SampleCartesianPoint(cartesian_trajectory, t, cartesian_point)) {
      ARM_LOG_ERROR("cartesian trajectory sample failed at t={}", t);
      return 1;
    }

    WriteJointRow(joint_csv, joint_point);
    WriteCartesianRow(cartesian_csv, cartesian_point);
    last_sample_time = joint_point.t;
  }

  const std::array<double, 3> sample_times{0.0, total_duration * 0.5,
                                           last_sample_time};

  ARM_LOG_INFO("Joint trajectory key samples");
  for (double t : sample_times) {
    ArmMotion::JointTrajectoryPoint_t point;
    if (!SampleJointPoint(joint_trajectory, t, point)) {
      ARM_LOG_ERROR("joint trajectory sample failed at t={}", t);
      return 1;
    }
    PrintJointPoint(point);
  }

  ARM_LOG_INFO("Cartesian trajectory samples");
  for (double t : sample_times) {
    ArmMotion::CartesianTrajectoryPoint_t point;
    if (!SampleCartesianPoint(cartesian_trajectory, t, point)) {
      ARM_LOG_ERROR("cartesian trajectory sample failed at t={}", t);
      return 1;
    }
    PrintCartesianPoint(point);
  }

  ARM_LOG_INFO("CSV exported: joint_pos.csv, cartesian.csv");
  ARM_LOG_INFO("Sampling step: {} s, duration: {} s", dt, total_duration);

  return 0;
}
