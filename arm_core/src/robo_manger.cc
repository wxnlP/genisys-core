#include "robo_manger.h"
#include "arm_logger.h"
#include "yaml-cpp/yaml.h"

RoboManager::RoboManager(const std::string& yaml_file) {
  std::string config_file_path = "./cfg/" + yaml_file;
  YAML::Node config = YAML::LoadFile(config_file_path);

  // 1. 处理 socketcan 配置
  std::string arm_port = config["robot"]["port"].as<std::string>();
  can_bus_ = std::make_shared<SocketCANBus>(arm_port, true);
  can_bus_->Init();

  // 2. 处理电机配置
  std::vector<uint32_t> feedback_ids;
  std::vector<double> motor_stiffness;
  std::vector<double> motor_damping;
  for (const auto& joint : config["robot"]["joints"]) {
    // 处理电机id和反馈id
    uint32_t motor_id = joint["motor_id"].as<uint32_t>();
    uint32_t feedback_id = joint["feedback_id"].as<uint32_t>();
    if (feedback_id != motor_id + 0x10) {
      ARM_LOG_WARN("推荐feedback_id 配置为 motor_id + 0x10, 当前 motor_id: {}, feedback_id: {} 会导致电机驱动反馈异常",
                   motor_id, feedback_id);
    }
    feedback_ids.push_back(feedback_id);
    // 处理电机型号
    std::string model_str = joint["model"].as<std::string>();
    DmMotorDriver::MotorModel motor_model;
    if (model_str == "DM4340P") {
      motor_model = DmMotorDriver::MotorModel::DM4340P;
    } else if (model_str == "DM4310") {
      motor_model = DmMotorDriver::MotorModel::DM4310;
    } else {
      ARM_LOG_ERROR("机械臂本体电机型号目前仅支持 'DM4340P' 或 'DM4310',请检查配置文件,当前值: {}", model_str);
    }
    // 处理电机刚度和阻尼
    double stiffness = joint["MIT"]["stiffness"].as<double>();
    double damping = joint["MIT"]["damping"].as<double>();
    motor_stiffness.push_back(stiffness);
    motor_damping.push_back(damping);

    arm_motors_.push_back(DmMotorDriver(*can_bus_, motor_id, motor_model));
  }

  if (arm_motors_.size() < 6) {
    ARM_LOG_ERROR("请检查配置文件，至少需要6个电机配置");
  }

  // 3.创建机械臂控制器
  arm_ctrl_ = std::make_shared<GeniArmCtrl>(arm_motors_);

  // 4.初始化机械臂实例
  // 处理机械臂的MDH参数和IK模式
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
  std::string ik_mode_str = config["robot"]["kinematics"]["ik_mode"].as<std::string>();
  KinematicSolver::IKMode ik_mode;
  if (ik_mode_str == "Piper") {
    ik_mode = KinematicSolver::IKMode::SphericalWrist;
  } else if (ik_mode_str == "UR") {
    ik_mode = KinematicSolver::IKMode::ParallelWrist;
  } else {
    ARM_LOG_ERROR("IK模式应为 'Piper' 或 'UR',请检查配置文件,当前值: {}", ik_mode_str);
  }
  // 创建机械臂实例
  arm_instance_ = std::make_shared<ArmInstance>(*arm_ctrl_, mdh_params, ik_mode);
  // 设置 MIT 固定参数
  ArmMotion::Joint6Dof_t arm_stiffness{};
  ArmMotion::Joint6Dof_t arm_damping{};
  for (size_t i = 0; i < 6; ++i) {
    arm_stiffness.q[i] = motor_stiffness[i];
    arm_damping.q[i] = motor_damping[i];
  }
  arm_instance_->SetMotorParams(arm_stiffness, arm_damping);

  // 5. 创建夹爪实例
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
    ARM_LOG_ERROR("夹爪电机型号目前仅支持 'DM4340P' 或 'DM4310',请检查配置文件,当前值: {}", gripper_model_str);
  }

  uint32_t gripper_id = config["gripper"]["motor_id"].as<uint32_t>();
  uint32_t gripper_feedback_id = config["gripper"]["feedback_id"].as<uint32_t>();
  if (gripper_feedback_id != gripper_id + 0x10) {
    ARM_LOG_WARN("推荐feedback_id 配置为 motor_id + 0x10,当前 motor_id: {}, feedback_id: {} 会导致电机驱动反馈异常",
                 gripper_id, gripper_feedback_id);
  }
  feedback_ids.push_back(gripper_feedback_id);
  gripper_motor_ = std::make_shared<DmMotorDriver>(*can_bus_, gripper_id, gripper_motor_model);
  double gripper_max_range = config["gripper"]["max_range"].as<double>();
  double gripper_stiffness = config["gripper"]["MIT"]["stiffness"].as<double>();
  double gripper_damping = config["gripper"]["MIT"]["damping"].as<double>();
  gripper_ = std::make_shared<GeniArmGripper>(*gripper_motor_, gripper_stiffness, gripper_damping);
  gripper_->SetMaxRange(gripper_max_range);

  // 6.初始化
  can_bus_->SetReceiverId(feedback_ids.data(), feedback_ids.size());
  gripper_->Init();

  ARM_LOG_INFO("RoboManager 初始化完成");
}