#include "arm_manager.h"
#include "arm_logger.h"
#include "yaml-cpp/yaml.h"

ArmManager::ArmManager(const std::string& yaml_file) {
  std::string config_file_path = "./cfg/" + yaml_file;
  YAML::Node config = YAML::LoadFile(config_file_path);

  auto robot_cfg = config["robot"];

  // ============================================================
  //  1. 关节限位
  // ============================================================
  std::vector<double> limits_lower =
      robot_cfg["joint_limits"]["lower"].as<std::vector<double>>();
  std::vector<double> limits_upper =
      robot_cfg["joint_limits"]["upper"].as<std::vector<double>>();

  // ============================================================
  //  2. MDH 参数
  // ============================================================
  ArmMotion::MDHParams mdh_params{};
  for (size_t i = 0; i < 6; ++i) {
    double alpha = robot_cfg["kinematics"]["MDH"]["alpha"][i].as<double>();
    double a = robot_cfg["kinematics"]["MDH"]["a"][i].as<double>();
    double d = robot_cfg["kinematics"]["MDH"]["d"][i].as<double>();
    double offset = robot_cfg["kinematics"]["MDH"]["offset"][i].as<double>();
    mdh_params.alpha[i] = alpha * (M_PI / 180.0);
    mdh_params.a[i] = a;
    mdh_params.d[i] = d;
    mdh_params.theta[i] = offset * (M_PI / 180.0);
  }

  // ============================================================
  //  3. 逆解模式
  // ============================================================
  std::string ik_mode_str =
      robot_cfg["kinematics"]["ik_mode"].as<std::string>();
  KinematicSolver::IKMode ik_mode;
  if (ik_mode_str == "Piper") {
    ik_mode = KinematicSolver::IKMode::SphericalWrist;
  } else if (ik_mode_str == "UR") {
    ik_mode = KinematicSolver::IKMode::ParallelWrist;
  } else {
    ARM_LOG_ERROR("IK 模式应为 'Piper' 或 'UR'，当前值: {}", ik_mode_str);
  }

  // ============================================================
  //  4. 创建 ArmController
  // ============================================================
  arm_controller_ = std::make_shared<ArmController>(mdh_params, ik_mode);

  // ============================================================
  //  5. MIT 参数 & 关节限位写入
  // ============================================================
  ArmMotion::Joint6Dof arm_stiffness{};
  ArmMotion::Joint6Dof arm_damping{};
  ArmMotion::Joint6Dof joint_min{};
  ArmMotion::Joint6Dof joint_max{};

  int count = 0;
  for (const auto& joint : robot_cfg["joints"]) {
    arm_stiffness.q[count] = joint["MIT"]["stiffness"].as<double>();
    arm_damping.q[count] = joint["MIT"]["damping"].as<double>();
    joint_min.q[count] = limits_lower[count];
    joint_max.q[count] = limits_upper[count];
    count++;
  }
  arm_controller_->SetJointLimits(joint_min, joint_max);
  arm_controller_->SetMotorParams(arm_stiffness, arm_damping);

  // ============================================================
  //  6. 轨迹模式
  // ============================================================
  std::string traj_mode_str =
      robot_cfg["trajectory_mode"].as<std::string>();
  if (traj_mode_str == "quintic") {
    arm_controller_->SetTrajectoryMode(ArmController::TrajectoryMode::QUINTIC);
  } else if (traj_mode_str == "cubic") {
    arm_controller_->SetTrajectoryMode(ArmController::TrajectoryMode::CUBIC);
  } else if (traj_mode_str == "lspb") {
    arm_controller_->SetTrajectoryMode(ArmController::TrajectoryMode::LSPB);
  } else {
    ARM_LOG_WARN(
        "轨迹模式 '{}' 不在可选范围内 ['quintic', 'cubic', 'lspb']，默认使用 quintic",
        traj_mode_str);
  }

  // ============================================================
  //  7. 采样频率
  // ============================================================
  double sample_frequency = robot_cfg["sample_frequency"].as<double>();
  arm_controller_->SetSampleFrequency(sample_frequency);

  // ============================================================
  //  8. 夹爪控制器
  // ============================================================
  auto gripper_cfg = config["gripper"];
  double gripper_stiffness = gripper_cfg["MIT"]["stiffness"].as<double>();
  double gripper_damping = gripper_cfg["MIT"]["damping"].as<double>();
  double gripper_max_range = gripper_cfg["max_range"].as<double>();

  gripper_controller_ =
      std::make_shared<GripperController>(gripper_stiffness, gripper_damping);
  gripper_controller_->SetMaxRange(gripper_max_range);
  gripper_controller_->SetSampleFrequency(sample_frequency);

  // ============================================================
  //  8. 打印初始化参数
  // ============================================================
  {
    constexpr auto kSepH =
        "══════════════════════════════════════════════════════════";
    constexpr auto kSepM =
        "──────────────────────────────────────────────────────────";
    constexpr double kRad2Deg = 180.0 / M_PI;

    ARM_LOG_INFO("{}", kSepH);
    ARM_LOG_INFO("  ArmManager 初始化完成");
    ARM_LOG_INFO("{}", kSepM);

    // MDH
    ARM_LOG_INFO("  MDH Parameters                               IK: {}",
                 ik_mode_str);
    ARM_LOG_INFO("  {:>3} │ {:>9} {:>9} {:>9} {:>9}",
                 "Jnt", "alpha(°)", "a(mm)", "d(mm)", "theta(°)");
    ARM_LOG_INFO("  ────┼────────────────────────────────────────────");
    for (size_t i = 0; i < 6; ++i) {
      ARM_LOG_INFO("  {:>3} │ {:>9.3f} {:>9.3f} {:>9.3f} {:>9.3f}",
                   i + 1,
                   mdh_params.alpha[i] * kRad2Deg,
                   mdh_params.a[i] * 1000.0,
                   mdh_params.d[i] * 1000.0,
                   mdh_params.theta[i] * kRad2Deg);
    }

    // 限位 & MIT
    ARM_LOG_INFO("{}", kSepM);
    ARM_LOG_INFO("  Joint Limits & MIT Parameters");
    ARM_LOG_INFO("  {:>3} │ {:>9} {:>9} {:>9} {:>8}",
                 "Jnt", "lower", "upper", "stiff", "damp");
    ARM_LOG_INFO("  ────┼────────────────────────────────────────────");
    for (size_t i = 0; i < 6; ++i) {
      ARM_LOG_INFO("  {:>3} │ {:>9.3f} {:>9.3f} {:>9.3f} {:>8.3f}",
                   i + 1,
                   limits_lower[i], limits_upper[i],
                   arm_stiffness.q[i], arm_damping.q[i]);
    }

    // 轨迹 & 夹爪
    ARM_LOG_INFO("{}", kSepM);
    ARM_LOG_INFO("  Trajectory: {}", traj_mode_str);
    ARM_LOG_INFO("  Gripper:    range={:.3f}, K={:.1f}, D={:.1f}",
                 gripper_max_range, gripper_stiffness, gripper_damping);
    ARM_LOG_INFO("{}", kSepH);
  }
}

