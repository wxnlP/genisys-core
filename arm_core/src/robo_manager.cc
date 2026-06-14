#include "robo_manager.h"
#include "arm_logger.h"
#include "arm_ctrl_base.h"
#include "arm_instance.h"
#include "canfd_bus.h"
#include "dm_motor_driver.h"
#include "geniarm_ctrl.h"
#include "geniarm_gripper.h"
#include "gripper_ctrl_base.h"
#ifdef BUILD_SIM_SUPPORT
  #include "sim_ctrl.h"
  #include "sim_gripper.h"
#endif
#include "yaml-cpp/yaml.h"

// 仅隔离真机硬件依赖，arm_core 自身类型直接放在 robo_manager.h 中
class RoboManager::Impl {
 public:
  std::shared_ptr<SocketCANBus> can_bus_;
  std::vector<DmMotorDriver> arm_motors_;
  std::shared_ptr<DmMotorDriver> gripper_motor_;
};

RoboManager::RoboManager(const std::string& yaml_file)
    : impl_(std::make_unique<Impl>()) {
  std::string config_file_path = "./cfg/" + yaml_file;
  YAML::Node config = YAML::LoadFile(config_file_path);

  std::string env = config["robot"]["env"].as<std::string>();

  // 1. 解析共享参数（真机 & 仿真通用）

  std::vector<double> limits_lower = config["robot"]["joint_limits"]["lower"].as<std::vector<double>>();
  std::vector<double> limits_upper = config["robot"]["joint_limits"]["upper"].as<std::vector<double>>();

  // MDH 参数
  ArmMotion::MDHParams mdh_params{};
  for (size_t i = 0; i < 6; ++i) {
    double alpha = config["robot"]["kinematics"]["MDH"]["alpha"][i].as<double>();
    double a = config["robot"]["kinematics"]["MDH"]["a"][i].as<double>();
    double d = config["robot"]["kinematics"]["MDH"]["d"][i].as<double>();
    double offset = config["robot"]["kinematics"]["MDH"]["offset"][i].as<double>();
    mdh_params.alpha[i] = alpha * (M_PI / 180.0);
    mdh_params.a[i] = a;
    mdh_params.d[i] = d;
    mdh_params.theta[i] = offset * (M_PI / 180.0);
  }
  // 运动学逆解模式
  std::string ik_mode_str = config["robot"]["kinematics"]["ik_mode"].as<std::string>();
  KinematicSolver::IKMode ik_mode;
  if (ik_mode_str == "Piper") {
    ik_mode = KinematicSolver::IKMode::SphericalWrist;
  } else if (ik_mode_str == "UR") {
    ik_mode = KinematicSolver::IKMode::ParallelWrist;
  } else {
    ARM_LOG_ERROR("IK模式应为 'Piper' 或 'UR'，请检查配置文件，当前值: {}", ik_mode_str);
  }

  // 轨迹模式
  std::string traj_mode_str = config["robot"]["trajectory_mode"].as<std::string>();

  // 夹爪最大行程
  double gripper_max_range = config["gripper"]["max_range"].as<double>();

  // 2. 按 env 创建控制器和夹爪

  std::vector<double> motor_stiffness;
  std::vector<double> motor_damping;

  if (env == "real") {
    std::string arm_port = config["robot"]["port"].as<std::string>();
    impl_->can_bus_ = std::make_shared<SocketCANBus>(arm_port, true);
    impl_->can_bus_->Init();

    std::vector<uint32_t> feedback_ids;
    int count = 0;
    for (const auto& joint : config["robot"]["joints"]) {
      uint32_t motor_id = joint["motor_id"].as<uint32_t>();
      uint32_t feedback_id = joint["feedback_id"].as<uint32_t>();
      if (feedback_id != motor_id + 0x10) {
        ARM_LOG_WARN("推荐feedback_id 配置为 motor_id + 0x10，当前 motor_id: {}，feedback_id: {}",
                     motor_id, feedback_id);
      }
      feedback_ids.push_back(feedback_id);

      std::string model_str = joint["model"].as<std::string>();
      DmMotorDriver::MotorModel motor_model;
      if (model_str == "DM4340P") {
        motor_model = DmMotorDriver::MotorModel::DM4340P;
      } else if (model_str == "DM4310") {
        motor_model = DmMotorDriver::MotorModel::DM4310;
      } else {
        ARM_LOG_ERROR("电机型号目前仅支持 'DM4340P' 或 'DM4310'，请检查配置文件，当前值: {}", model_str);
      }

      double stiffness = joint["MIT"]["stiffness"].as<double>();
      double damping = joint["MIT"]["damping"].as<double>();
      motor_stiffness.push_back(stiffness);
      motor_damping.push_back(damping);

      impl_->arm_motors_.push_back(DmMotorDriver(*impl_->can_bus_, motor_id,
                                                  limits_lower[count], limits_upper[count], motor_model));
      count++;
    }

    if (impl_->arm_motors_.size() < 6) {
      ARM_LOG_ERROR("请检查配置文件，至少需要6个电机配置");
    }

    arm_ctrl_ = std::make_shared<GeniArmCtrl>(impl_->arm_motors_);

    // 夹爪
    std::string gripper_port = config["gripper"]["port"].as<std::string>();
    if (gripper_port != arm_port) {
      ARM_LOG_ERROR("当前版本仅支持机械臂和夹爪共用一个CAN接口，请检查配置文件");
    }
    std::string gripper_model_str = config["gripper"]["model"].as<std::string>();
    DmMotorDriver::MotorModel gripper_motor_model;
    if (gripper_model_str == "DM4340P") {
      gripper_motor_model = DmMotorDriver::MotorModel::DM4340P;
    } else if (gripper_model_str == "DM4310") {
      gripper_motor_model = DmMotorDriver::MotorModel::DM4310;
    } else {
      ARM_LOG_ERROR("夹爪电机型号目前仅支持 'DM4340P' 或 'DM4310'，请检查配置文件，当前值: {}", gripper_model_str);
    }

    uint32_t gripper_id = config["gripper"]["motor_id"].as<uint32_t>();
    uint32_t gripper_feedback_id = config["gripper"]["feedback_id"].as<uint32_t>();
    if (gripper_feedback_id != gripper_id + 0x10) {
      ARM_LOG_WARN("推荐feedback_id 配置为 motor_id + 0x10，当前 motor_id: {}，feedback_id: {}",
                   gripper_id, gripper_feedback_id);
    }
    feedback_ids.push_back(gripper_feedback_id);
    impl_->gripper_motor_ = std::make_shared<DmMotorDriver>(*impl_->can_bus_, gripper_id, gripper_motor_model);

    double gripper_stiffness = config["gripper"]["MIT"]["stiffness"].as<double>();
    double gripper_damping = config["gripper"]["MIT"]["damping"].as<double>();
    gripper_ = std::make_shared<GeniArmGripper>(*impl_->gripper_motor_, gripper_stiffness, gripper_damping);

    impl_->can_bus_->SetReceiverId(feedback_ids.data(), feedback_ids.size());
    gripper_->Init();

  }
#ifdef BUILD_SIM_SUPPORT
  else if (env == "sim") {
    for (size_t i = 0; i < 6; ++i) {
      motor_stiffness.push_back(config["robot"]["MIT"]["stiffness"][i].as<double>());
      motor_damping.push_back(config["robot"]["MIT"]["damping"][i].as<double>());
    }

    arm_ctrl_ = std::make_shared<SimCtrl>();

    double gripper_stiffness = config["gripper"]["MIT"]["stiffness"].as<double>();
    double gripper_damping = config["gripper"]["MIT"]["damping"].as<double>();
    gripper_ = std::make_shared<SimGripper>(gripper_stiffness, gripper_damping);
  }
#endif
  else {
    ARM_LOG_ERROR("env 应为 'real' 或 'sim'，当前值: {}", env);
  }

  // 3. 创建 ArmInstance 并初始化
  arm_instance_ = std::make_shared<ArmInstance>(*arm_ctrl_, mdh_params, ik_mode);

  gripper_->SetMaxRange(gripper_max_range);

  ArmMotion::Joint6Dof_t arm_stiffness{};
  ArmMotion::Joint6Dof_t arm_damping{};
  ArmMotion::Joint6Dof_t joint_min{};
  ArmMotion::Joint6Dof_t joint_max{};
  for (size_t i = 0; i < 6; ++i) {
    arm_stiffness.q[i] = motor_stiffness[i];
    arm_damping.q[i] = motor_damping[i];
    joint_min.q[i] = limits_lower[i];
    joint_max.q[i] = limits_upper[i];
  }
  arm_instance_->SetJointLimits(joint_min, joint_max);
  arm_instance_->SetMotorParams(arm_stiffness, arm_damping);

  if (traj_mode_str == "quintic") {
    arm_instance_->SetTrajectoryMode(ArmInstance::TrajectoryMode::QUINTIC);
  } else if (traj_mode_str == "cubic") {
    arm_instance_->SetTrajectoryMode(ArmInstance::TrajectoryMode::CUBIC);
  } else if (traj_mode_str == "lspb") {
    arm_instance_->SetTrajectoryMode(ArmInstance::TrajectoryMode::LSPB);
  } else {
    ARM_LOG_WARN("'{}'不在可选范围内：['quintic', 'cubic', 'lspb']，默认使用quintic", traj_mode_str);
  }

  arm_instance_->Init();

  // 打印初始化参数
  {
    constexpr auto kSepH =
        "══════════════════════════════════════════════════════════";
    constexpr auto kSepM =
        "──────────────────────────────────────────────────────────";
    constexpr double kRad2Deg = 180.0 / M_PI;

    ARM_LOG_INFO("{}", kSepH);
    ARM_LOG_INFO("  RoboManager 初始化完成 [env: {}]", env);
    ARM_LOG_INFO("{}", kSepM);

    // --- MDH 参数 ---
    ARM_LOG_INFO("  MDH  Parameters                                   IK: {}",
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

    // --- 限位 & MIT ---
    ARM_LOG_INFO("{}", kSepM);
    ARM_LOG_INFO("  Joint Limits & MIT Parameters");
    ARM_LOG_INFO("  {:>3} │ {:>9} {:>9} {:>9} {:>8}",
                 "Jnt", "lower", "upper", "stiff", "damp");
    ARM_LOG_INFO("  ────┼────────────────────────────────────────────");

    for (size_t i = 0; i < 6; ++i) {
      ARM_LOG_INFO("  {:>3} │ {:>9.3f} {:>9.3f} {:>9.3f} {:>8.3f}",
                   i + 1,
                   limits_lower[i],
                   limits_upper[i],
                   motor_stiffness[i],
                   motor_damping[i]);
    }

    // --- 轨迹 & 夹爪 ---
    double g_stiff = config["gripper"]["MIT"]["stiffness"].as<double>();
    double g_damp = config["gripper"]["MIT"]["damping"].as<double>();
    ARM_LOG_INFO("{}", kSepM);
    ARM_LOG_INFO("  Trajectory: {}", traj_mode_str);
    ARM_LOG_INFO("  Gripper:    range={:.3f}, K={:.1f}, D={:.1f}",
                 gripper_max_range, g_stiff, g_damp);
    ARM_LOG_INFO("{}", kSepH);
  }
}

RoboManager::~RoboManager() = default;

RoboManager::RoboManager(RoboManager&&) noexcept = default;
RoboManager& RoboManager::operator=(RoboManager&&) noexcept = default;
