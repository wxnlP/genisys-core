#include <atomic>
#include <csignal>
#include <future>
#include "arm_logger.h"
#include "robo_manager.h"

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

int main() {
  std::signal(SIGINT, SignalHandler);  // 注册 Ctrl+C 信号
  RoboManager robo_manger("robotic_arm_config.yaml");
  std::shared_ptr<ArmInstance> arm = robo_manger.GetArmInstance();
  std::shared_ptr<GeniArmGripper> gripper = robo_manger.GetGripper();

  arm->Enable();
  // gripper->Enable();

  
  std::jthread arm_joint_thread([&arm, &gripper](std::stop_token stoken) {
    ArmMotion::Pose6Dof_t pose{};
    ArmMotion::Joint6Dof_t jog{};
    double gripper_pos{};
    double gripper_vel{};
    double gripper_effort{};
    while (!stoken.stop_requested()) {
      // if (arm->GetCurrentPose(pose) == -1) {
      //   ARM_LOG_WARN("获取姿态失败");
      // }
      // ARM_LOG_INFO("\n---\n x: {}, y: {}, z: {}",
      //               pose.x, pose.y, pose.z);
      // gripper->GetPosition(gripper_pos, gripper_effort);
      if (arm->GetCurrentJoint(jog) == -1) {
        ARM_LOG_WARN("获取关节角失败");
      }
      ARM_LOG_INFO("\n-----\n j1: {:.4f}, j2: {:.4f}, j3: {:.4f}, j4: {:.4f}, j5: {:.4f}, j6: {:.4f}\n-----", 
                    jog.q[0], jog.q[1], jog.q[2], jog.q[3], jog.q[4], jog.q[5]);
      // gripper->ReceiveState(gripper_pos, gripper_vel, gripper_effort);
      // ARM_LOG_INFO("\n-----\n gripper_pos: {}, gripper_effort: {}\n-----", gripper_pos, gripper_effort);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  /* 任务测试 */
  // ARM_LOG_INFO("开始夹爪测试");
  // gripper->SetPosition(1, 0.5, 1.0, 0);
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  // gripper->SetPosition(0, 0.5, 1.0, 0);

  ARM_LOG_INFO("开始关节Move测试");
  ArmMotion::Joint6Dof_t jog{ 0, 1.00, -1.00, 0.0, 0.0, 0.0};
  arm->MoveJ(jog, 0.5, 0.3, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  ArmMotion::Pose6Dof_t current_pose;
  arm->GetCurrentPose(current_pose);
  ARM_LOG_INFO("当前位置：x: {}, y: {}, z: {}", current_pose.x, current_pose.y, current_pose.z);

  // ArmMotion::Pose6Dof_t traget_pose = current_pose;
  // traget_pose.y += 0.1;
  // ARM_LOG_INFO("朝Y方向平移0.1m");
  // arm->MoveL(traget_pose, 0.5, 0.3, 0);
  // arm->GetCurrentPose(current_pose);
  // ARM_LOG_INFO("当前位置：x: {}, y: {}, z: {}", current_pose.x, current_pose.y, current_pose.z);

  std::vector<ArmMotion::Pose6Dof_t> cart_points;
  ArmMotion::Pose6Dof_t p1 = current_pose;
  p1.y -= 0.10;
  cart_points.push_back(p1);

  ArmMotion::Pose6Dof_t p2 = p1;
  p2.z -= 0.10;
  cart_points.push_back(p2);

  ArmMotion::Pose6Dof_t p3 = p2;
  p3.y += 0.10;
  cart_points.push_back(p3);

  ArmMotion::Pose6Dof_t p4 = p3;
  p4.z += 0.10;
  cart_points.push_back(p4);

  arm->MoveLMulti(cart_points, 0.01, 0.1, 0.2, 0);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  jog = ArmMotion::Joint6Dof_t{};
  arm->MoveJ(jog, 0.5, 0.3, 0);

  /* 处理 Ctrl + C 信号 */
  while (g_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ARM_LOG_INFO("回收线程中......");

  arm_joint_thread.request_stop();
  arm_joint_thread.join();

  ARM_LOG_INFO("回收线程完成");

  ARM_LOG_INFO("失能机械臂");

  arm->Disable();
  // gripper->Disable();

  return 0;
}