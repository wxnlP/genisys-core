#include "actuator_manager.h"
#include <yaml-cpp/yaml.h>
#include "geniarm_ctrl.h"
#include "geniarm_gripper.h"
#include "s6h4d_ctrl.h"

ActuatorManager::ActuatorManager(std::string file) {
  std::string config_file_path = "./cfg/" + file;
  YAML::Node config = YAML::LoadFile(config_file_path);

  auto actuator = config["actuator"];

  // ============================================================
  //  机械臂 (Arm)
  // ============================================================
  auto arm_cfg = actuator["arm"];
  std::string arm_name = arm_cfg["name"].as<std::string>();

  std::vector<uint32_t> feedback_ids;

  if (arm_name == "geniarm") {
    std::string arm_port = arm_cfg["protocol"]["port"].as<std::string>();
    std::vector<double> limits_lower = arm_cfg["joint_limits"]["lower"].as<std::vector<double>>();
    std::vector<double> limits_upper = arm_cfg["joint_limits"]["upper"].as<std::vector<double>>();
    for (auto& v : limits_lower) v *= M_PI / 180.0;
    for (auto& v : limits_upper) v *= M_PI / 180.0;

    // 创建 CAN 总线
    if (!can_bus_) {
      can_bus_ = std::make_shared<SocketCANBus>(arm_port, true);
      can_bus_->Init();
    }

    // 解析各关节电机
    int count = 0;
    joint_names_.clear();
    for (const auto& joint : arm_cfg["joints"]) {
      uint32_t motor_id = joint["motor_id"].as<uint32_t>();
      uint32_t feedback_id = joint["feedback_id"].as<uint32_t>();
      feedback_ids.push_back(feedback_id);
      joint_names_.push_back(joint["name"].as<std::string>());

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

    arm_ctrl_ = std::make_shared<GeniArmCtrl>(arm_motors_);

    // 设置控制模式（默认 MIT）
    std::string ctrl_mode_str = arm_cfg["ctrl_mode"].as<std::string>();
    if (ctrl_mode_str == "POS") {
      arm_ctrl_->SetCtrlMode(ArmCtrlBase::CtrlMode::POS);
    } else if (ctrl_mode_str == "VEL") {
      arm_ctrl_->SetCtrlMode(ArmCtrlBase::CtrlMode::VEL);
    }
    // "MIT" 或不指定时保持默认

  } else if (arm_name == "s6h4d") {
    auto proto = arm_cfg["protocol"];
    std::string arm_port = proto["port"].as<std::string>();
    int baud_rate = proto["baud_rate"].as<int>();
    // 记录关节名称
    for (const auto& joint : arm_cfg["joints"]) {
      joint_names_.push_back(joint["name"].as<std::string>());
    }
    // 创建串口
    serial_ = std::make_unique<serial::Serial>(arm_port, baud_rate, serial::Timeout::simpleTimeout(100));
    arm_ctrl_ = std::make_shared<S6h4dCtrl>(*serial_);
  }

  arm_ctrl_->Init();

  // ============================================================
  //  夹爪 (Gripper) — 仅在配置中存在时初始化
  // ============================================================
  if (actuator["gripper"]) {
    auto gripper_cfg = actuator["gripper"];
    std::string gripper_name = gripper_cfg["name"].as<std::string>();
    joint_names_.push_back(gripper_name);

    if (gripper_name == "geniarm_gripper") {
      if (!can_bus_) {
        std::string gripper_port = gripper_cfg["protocol"]["port"].as<std::string>();
        can_bus_ = std::make_shared<SocketCANBus>(gripper_port, true);
        can_bus_->Init();
      }

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
      gripper_motor_ = std::make_shared<DmMotorDriver>(*can_bus_, gripper_motor_id, gripper_motor_model);
      gripper_ctrl_ = std::make_shared<GeniArmGripper>(*gripper_motor_);
    }

    gripper_ctrl_->Init();
  }

  // 注册所有反馈帧 ID
  if (can_bus_) {
    can_bus_->SetReceiverId(feedback_ids.data(), static_cast<uint8_t>(feedback_ids.size()));
  }
}

void ActuatorManager::Shutdown() {
  // 通知各内部线程退出，实际 join 和资源释放由各 jthread 析构函数完成
  if (arm_ctrl_)     arm_ctrl_->Shutdown();
  if (gripper_ctrl_) gripper_ctrl_->Shutdown();
  if (can_bus_)      can_bus_->Shutdown();
}
