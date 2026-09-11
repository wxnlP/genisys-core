#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <thread>
#include <vector>

#include "actuator_manager.h"
#include "arm_controller.h"
#include "arm_logger.h"
#include "arm_manager.h"
#include "gripper_controller.h"

std::atomic<bool> g_running{true};
void SignalHandler(int) { g_running = false; }

int main() {
  std::signal(SIGINT, SignalHandler);

  ActuatorManager actuator_manager("geniarm_actuator_cfg.yaml");
  ArmManager arm_manager("geniarm_body_cfg.yaml");

  auto arm_ctrl = actuator_manager.GetArmActuator();
  auto gripper_ctrl = actuator_manager.GetGripperActuator();
  auto arm = arm_manager.GetArmController();
  auto gripper = arm_manager.GetGripperController();

  arm_ctrl->Enable();
  if (gripper_ctrl) gripper_ctrl->Enable();

  // 控制回路：读反馈 → 取指令 → 下发；无轨迹时保持当前姿态
  std::jthread loop([&](std::stop_token stoken) {
    auto next = std::chrono::steady_clock::now();
    while (!stoken.stop_requested() && g_running) {
      ArmCtrlBase::JointState actuator_state;
      if (!arm_ctrl->GetJointState(actuator_state)) {
        ARM_LOG_WARN("读取关节状态失败，跳过本周期");
        next += std::chrono::milliseconds(5);
        std::this_thread::sleep_until(next);
        continue;
      }
      {
        ArmMotion::JointState motion_state;
        std::memcpy(&motion_state, &actuator_state, sizeof(motion_state));
        arm->SetJointState(motion_state);
      }
      ArmMotion::JointCommand motion_cmd;
      ArmCtrlBase::JointCommand actuator_cmd;
      if (arm->GetJointCommand(motion_cmd)) {
        std::memcpy(&actuator_cmd, &motion_cmd, sizeof(actuator_cmd));
      } else {
        actuator_cmd = ArmCtrlBase::JointCommand{};
        std::memcpy(&actuator_cmd.position, &actuator_state.position, sizeof(actuator_cmd.position));
        std::memcpy(&actuator_cmd.stiffness, &motion_cmd.stiffness, sizeof(actuator_cmd.stiffness));
        std::memcpy(&actuator_cmd.damping, &motion_cmd.damping, sizeof(actuator_cmd.damping));
      }
      arm_ctrl->SetJointState(actuator_cmd);

      if (gripper_ctrl) {
        GripperCtrlBase::GripperState gripper_state;
        if (gripper_ctrl->ReceiveState(gripper_state)) {
          gripper->SetGripperState(gripper_state.position, gripper_state.velocity, gripper_state.effort);
        }
        GripperController::GripperCommand gripper_cmd;
        if (gripper->GetGripperCommand(gripper_cmd) == 0) {
          GripperCtrlBase::GripperCommand out;
          out.position = gripper_cmd.position;
          out.velocity = gripper_cmd.velocity;
          out.effort = gripper_cmd.effort;
          out.stiffness = gripper_cmd.stiffness;
          out.damping = gripper_cmd.damping;
          gripper_ctrl->SendCommand(out);
        }
      }
      next += std::chrono::milliseconds(5);  // 200 Hz
      std::this_thread::sleep_until(next);
    }
  });

  // 关节角监视
  std::jthread monitor([&](std::stop_token stoken) {
    while (!stoken.stop_requested() && g_running) {
      ArmMotion::Joint6Dof jog;
      if (arm->GetCurrentJoint(jog) == 0) {
        ARM_LOG_INFO("j1: {:.4f}, j2: {:.4f}, j3: {:.4f}, j4: {:.4f}, j5: {:.4f}, j6: {:.4f}",
                     jog.q[0], jog.q[1], jog.q[2], jog.q[3], jog.q[4], jog.q[5]);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  if (gripper) {
    ARM_LOG_INFO("开始夹爪测试");
    gripper->PosControl(1.0, 0.5, 1.0, 0.0);
    gripper->WaitForCompletion();
    gripper->PosControl(0.0, 0.5, 1.0, 0.0);
    gripper->WaitForCompletion();
  }

  ARM_LOG_INFO("开始关节 MoveJ 测试");
  arm->MoveJ(ArmMotion::Joint6Dof{0, 1.00, -1.00, 0.0, 0.0, 0.0}, 0.5, 0.3);
  arm->WaitForCompletion();

  ArmMotion::Pose6Dof current_pose;
  if (arm->GetCurrentPose(current_pose) == 0) {
    ARM_LOG_INFO("当前位置：x: {:.4f}, y: {:.4f}, z: {:.4f}", current_pose.x, current_pose.y, current_pose.z);
  }

  ARM_LOG_INFO("开始笛卡尔多点 MoveL 测试");
  std::vector<ArmMotion::Pose6Dof> cart_points;
  ArmMotion::Pose6Dof p1 = current_pose; p1.y -= 0.10; cart_points.push_back(p1);
  ArmMotion::Pose6Dof p2 = p1;           p2.z -= 0.10; cart_points.push_back(p2);
  ArmMotion::Pose6Dof p3 = p2;           p3.y += 0.10; cart_points.push_back(p3);
  ArmMotion::Pose6Dof p4 = p3;           p4.z += 0.10; cart_points.push_back(p4);
  arm->MoveLMulti(cart_points, 0.05, 0.1, 0.2);
  arm->WaitForCompletion();

  while (g_running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  ARM_LOG_INFO("回收线程中......");
  monitor.request_stop();
  loop.request_stop();
  loop.join();
  monitor.join();

  ARM_LOG_INFO("失能执行器");
  arm_ctrl->Disable();
  if (gripper_ctrl) gripper_ctrl->Disable();

  std::signal(SIGINT, SIG_DFL);  // 恢复默认，退出阶段Ctrl+C才有效
  actuator_manager.Shutdown();   // 停CAN采样线程
  return 0;
}
