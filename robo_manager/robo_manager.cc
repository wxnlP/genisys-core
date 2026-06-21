#include "robo_manager.h"

#include <iostream>
#include <cstring>

#include "arm_logger.h"
#include "yaml-cpp/yaml.h"

RoboManager::RoboManager(const std::string& config_yaml) {
  std::string config_path = "./cfg/" + config_yaml;
  YAML::Node config = YAML::LoadFile(config_path);

  std::string actuator_yaml = config["actuator_config"].as<std::string>();
  std::string arm_yaml = config["arm_config"].as<std::string>();
  double frequency = config["control_frequency"].as<double>();
  dt_ = 1.0 / frequency;

  // 1. 创建管理器
  actuator_mgr_ = std::make_shared<ActuatorManager>(actuator_yaml);
  arm_mgr_ = std::make_shared<ArmManager>(arm_yaml);

  // 2. 取出驱动层指针
  actuator_ = actuator_mgr_->GetArmController();
  gripper_actuator_ = actuator_mgr_->GetGripperController();

  // 3. 取出算法层指针
  arm_controller_ = arm_mgr_->GetArmController();
  gripper_controller_ = arm_mgr_->GetGripperController();

  ARM_LOG_INFO("[RoboManager] 初始化完成, 控制频率 {:.0f} Hz, dt={:.4f} s",
               frequency, dt_);
}

RoboManager::~RoboManager() { Stop(); }


void RoboManager::Start() {
  if (state_ == State::DISABLED) {
    ARM_LOG_ERROR("[RoboManager] Start 失败：未使能");
    return;
  }
  if (control_thread_ && control_thread_->joinable()) {
    ARM_LOG_WARN("[RoboManager] Start：已在运行中");
    return;
  }
  stop_requested_ = false;
  control_thread_ = std::make_unique<std::jthread>(
      [this](std::stop_token /*stoken*/) { ControlLoop(); });
  ARM_LOG_INFO("[RoboManager] 控制循环启动，dt={:.4f} s", dt_);
}

void RoboManager::Stop() {
  if (!control_thread_) return;

  ARM_LOG_INFO("[RoboManager] 正在停止控制循环...");
  stop_requested_ = true;
  control_thread_->request_stop();
  if (control_thread_->joinable()) {
    control_thread_->join();
  }
  control_thread_.reset();
  state_ = State::IDLE;
  ARM_LOG_INFO("[RoboManager] 控制循环已停止");
}

void RoboManager::EmergencyStop() {
  stop_requested_ = true;

  // 发送零指令到所有电机
  ArmCtrlBase::JointCommand cmd{};
  actuator_->SetJointState(cmd);

  state_ = State::IDLE;
  ARM_LOG_WARN("[RoboManager] 紧急停机");
}


RoboManager::State RoboManager::GetState() const {
  return state_.load();
}

bool RoboManager::IsMoving() const {
  return state_ == State::MOVING;
}


void RoboManager::Enable() {
  actuator_->Enable();
  gripper_actuator_->Enable();
  state_ = State::IDLE;
  ARM_LOG_INFO("[RoboManager] 已使能");
}

void RoboManager::Disable() {
  Stop();
  actuator_->Disable();
  gripper_actuator_->Disable();
  state_ = State::DISABLED;
  ARM_LOG_INFO("[RoboManager] 已去使能");
}

void RoboManager::SetCycleFrequency(double hz) {
  dt_ = 1.0 / hz;
  ARM_LOG_INFO("[RoboManager] 控制频率设为 {:.1f} Hz, dt={:.4f} s", hz, dt_);
}


std::shared_ptr<ArmController> RoboManager::GetArmController() {
  return arm_controller_;
}

std::shared_ptr<GripperController> RoboManager::GetGripperController() {
  return gripper_controller_;
}


void RoboManager::ControlLoop() {

  while (!stop_requested_) {
    auto t_start = std::chrono::steady_clock::now();

    // ── 1. 读取机械臂关节反馈 ──
    ArmCtrlBase::JointState actuator_state;
    if (!actuator_->GetJointState(actuator_state)) {
      ARM_LOG_ERROR("[RoboManager] GetJointState 失败");
      state_ = State::ERROR;
      WaitNextCycle(t_start);
      continue;
    }

    // 更新状态
    ArmMotion::JointState motion_state;
    std::memcpy(&motion_state, &actuator_state, sizeof(motion_state));
    arm_controller_->SetJointState(motion_state);

    // ── 2. 消费机械臂指令队列 ──
    ArmMotion::JointCommand motion_cmd;
    if (arm_controller_->GetJointCommand(motion_cmd)) {
      state_ = State::MOVING;
      ArmCtrlBase::JointCommand actuator_cmd;
      std::memcpy(&actuator_cmd, &motion_cmd, sizeof(actuator_cmd));
      actuator_->SetJointState(actuator_cmd);
    } else {
      state_ = State::IDLE;
    }

    // ── 3. 读取夹爪反馈 ──
    GripperCtrlBase::GripperState gripper_state;
    if (gripper_actuator_->ReceiveState(gripper_state)) {
      // 更新状态
      gripper_controller_->SetGripperState(gripper_state.position,
                                           gripper_state.velocity,
                                           gripper_state.effort);
    }

    // ── 4. 消费夹爪指令队列 ──
    GripperController::GripperCommand gripper_cmd;
    if (gripper_controller_->GetGripperCommand(gripper_cmd) == 0) {
      GripperCtrlBase::GripperCommand actuator_gripper_cmd;
      actuator_gripper_cmd.position = gripper_cmd.position;
      actuator_gripper_cmd.velocity = gripper_cmd.velocity;
      actuator_gripper_cmd.effort = gripper_cmd.effort;
      actuator_gripper_cmd.stiffness = gripper_cmd.stiffness;
      actuator_gripper_cmd.damping = gripper_cmd.damping;
      gripper_actuator_->SendCommand(actuator_gripper_cmd);
    }

    WaitNextCycle(t_start);
  }
}

void RoboManager::WaitNextCycle(std::chrono::steady_clock::time_point t_start) {
  auto target = t_start + std::chrono::duration<double>(dt_);
  auto now = std::chrono::steady_clock::now();

  // ── 掉帧检测 ──
  if (now > target) {
    overrun_counter_++;
    // 前 3 帧每次都报，之后每 100 帧报一次，避免刷屏
    if (overrun_counter_ <= 3 || overrun_counter_ % 100 == 0) {
      ARM_LOG_WARN("[RoboManager] 掉帧: 滞后 {:.0f} μs (连续 {} 帧)",
                   (now - target).count() / 1e3, overrun_counter_);
    }
  } else {
    overrun_counter_ = 0;
  }

  // 粗粒度 sleep
  if (target - now > std::chrono::microseconds(200)) {
    std::this_thread::sleep_until(target - std::chrono::microseconds(150));
  }

  // 细粒度自旋
  while (std::chrono::steady_clock::now() < target) {
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__)
    __builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
    __asm__ volatile("yield");
#endif
  }
}

