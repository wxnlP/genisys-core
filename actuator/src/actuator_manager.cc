#include "actuator_manager.h"
#include "geniarm_ctrl.h"
#include "geniarm_gripper.h"
#include <yaml-cpp/yaml.h>

ActuatorManager::ActuatorManager(std::string file) {
  std::string config_file_path = "./cfg/" + file;
  YAML::Node config = YAML::LoadFile(config_file_path);

  auto actuator = config["actuator"];

  // ============================================================
  //  机械臂 (Arm)
  // ============================================================
  auto arm_cfg = actuator["arm"];
  std::string arm_name = arm_cfg["name"].as<std::string>();
  std::string arm_port = arm_cfg["port"].as<std::string>();

  std::vector<double> limits_lower = arm_cfg["joint_limits"]["lower"].as<std::vector<double>>();
  std::vector<double> limits_upper = arm_cfg["joint_limits"]["upper"].as<std::vector<double>>();

  // 创建 CAN 总线
  can_bus_ = std::make_shared<SocketCANBus>(arm_port, true);
  can_bus_->Init();

  // 解析各关节电机
  std::vector<uint32_t> feedback_ids;
  int count = 0;
  for (const auto& joint : arm_cfg["joints"]) {
    uint32_t motor_id = joint["motor_id"].as<uint32_t>();
    uint32_t feedback_id = joint["feedback_id"].as<uint32_t>();
    feedback_ids.push_back(feedback_id);

    std::string model_str = joint["model"].as<std::string>();
    DmMotorDriver::MotorModel motor_model;
    if (model_str == "DM4340P") {
      motor_model = DmMotorDriver::MotorModel::DM4340P;
    } else if (model_str == "DM4310") {
      motor_model = DmMotorDriver::MotorModel::DM4310;
    }

    arm_motors_.emplace_back(*can_bus_, motor_id,
                             limits_lower[count], limits_upper[count],
                             motor_model);
    count++;
  }

  // 按名称创建机械臂控制器
  if (arm_name == "geniarm") {
    arm_ctrl_ = std::make_shared<GeniArmCtrl>(arm_motors_);
  }
  arm_ctrl_->Init();

  // ============================================================
  //  夹爪 (Gripper)
  // ============================================================
  auto gripper_cfg = actuator["gripper"];
  std::string gripper_name = gripper_cfg["name"].as<std::string>();

  std::string gripper_model_str = gripper_cfg["model"].as<std::string>();
  DmMotorDriver::MotorModel gripper_motor_model;
  if (gripper_model_str == "DM4340P") {
    gripper_motor_model = DmMotorDriver::MotorModel::DM4340P;
  } else if (gripper_model_str == "DM4310") {
    gripper_motor_model = DmMotorDriver::MotorModel::DM4310;
  }

  uint32_t gripper_motor_id = gripper_cfg["motor_id"].as<uint32_t>();
  uint32_t gripper_feedback_id = gripper_cfg["feedback_id"].as<uint32_t>();
  feedback_ids.push_back(gripper_feedback_id);
  gripper_motor_ = std::make_shared<DmMotorDriver>(*can_bus_, gripper_motor_id,
                                                   gripper_motor_model);

  if (gripper_name == "geniarm_gripper") {
    gripper_ctrl_ = std::make_shared<GeniArmGripper>(*gripper_motor_);
  }

  // 注册所有反馈帧 ID 并初始化夹爪
  can_bus_->SetReceiverId(feedback_ids.data(),
                          static_cast<uint8_t>(feedback_ids.size()));
  gripper_ctrl_->Init();
}

void ActuatorManager::Shutdown() {
  // 通知各内部线程退出，实际 join 和资源释放由各 jthread 析构函数完成
  if (arm_ctrl_)     arm_ctrl_->Shutdown();
  if (gripper_ctrl_) gripper_ctrl_->Shutdown();
  if (can_bus_)      can_bus_->Shutdown();
}
