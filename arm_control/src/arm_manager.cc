#include "arm_manager.h"
#include <Eigen/Geometry>
#include "arm_logger.h"
#include "yaml-cpp/yaml.h"

namespace {

// ------------------------------------------------------------------
// 末端工具（tool）惯量融合：把刚性安装在连杆上的工具并入该连杆参数
//   m = m_l + m_t
//   c = (m_l·c_l + m_t·c_t) / m
//   I = Σ_i [ I_i + m_i·(|d_i|²·E − d_i·d_iᵀ) ],  d_i = c_i − c
// 前提：link.com / tool.com 均以「该连杆坐标系」表达；
//       I 均为「绕各自质心、与连杆系轴对齐」的张量（YAML 约定）。
// 融合后求解器（RNEA 等）读到的就是"变重并偏心的最后一根连杆"，无需改动。
// ------------------------------------------------------------------
void FuseToolIntoLink(Dynamics::Link& link, double tool_mass, const double tool_com[3],
                      const double tool_inertia[9]) {
  const double m_l = link.mass;
  const double m = m_l + tool_mass;

  const Eigen::Vector3d c_l = Eigen::Map<const Eigen::Vector3d>(link.com);
  const Eigen::Vector3d c_t(tool_com[0], tool_com[1], tool_com[2]);
  const Eigen::Vector3d c = (m_l * c_l + tool_mass * c_t) / m;

  Eigen::Matrix3d I_l;
  I_l << link.inertia[0], link.inertia[1], link.inertia[2],
         link.inertia[3], link.inertia[4], link.inertia[5],
         link.inertia[6], link.inertia[7], link.inertia[8];
  Eigen::Matrix3d I_t;
  I_t << tool_inertia[0], tool_inertia[1], tool_inertia[2],
         tool_inertia[3], tool_inertia[4], tool_inertia[5],
         tool_inertia[6], tool_inertia[7], tool_inertia[8];

  auto parallel_axis = [](double m_i, const Eigen::Vector3d& d) {
    return m_i * (d.squaredNorm() * Eigen::Matrix3d::Identity() - d * d.transpose());
  };
  const Eigen::Matrix3d I =
      I_l + parallel_axis(m_l, c_l - c) + I_t + parallel_axis(tool_mass, c_t - c);

  link.mass = m;
  for (int i = 0; i < 3; ++i) {
    link.com[i] = c[i];
  }
  for (int r = 0; r < 3; ++r) {
    for (int col = 0; col < 3; ++col) {
      link.inertia[r * 3 + col] = I(r, col);
    }
  }
}

}  // namespace

ArmManager::ArmManager(const std::string& yaml_file) {
  std::string config_file_path = "./cfg/" + yaml_file;
  YAML::Node config = YAML::LoadFile(config_file_path);

  auto robot_cfg = config["robot"];
  std::string robot_name = robot_cfg["name"].as<std::string>();

  // ============================================================
  //  1. 关节限位（YAML 使用角度，内部转为弧度）
  // ============================================================
  std::vector<double> limits_lower = robot_cfg["joint_limits"]["lower"].as<std::vector<double>>();
  std::vector<double> limits_upper = robot_cfg["joint_limits"]["upper"].as<std::vector<double>>();
  for (auto& v : limits_lower) v *= M_PI / 180.0;
  for (auto& v : limits_upper) v *= M_PI / 180.0;

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
  std::string ik_mode_str = robot_cfg["kinematics"]["ik_mode"].as<std::string>();
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
  arm_controller_->SetRobotName(robot_name);

  // Tool 配置缓存（用于后续统一打印）
  bool tool_configured = false;
  std::string tool_frame = "tcp";
  double tool_xyz_cache[3] = {0.0, 0.0, 0.0};
  double tool_rpy_cache_deg[3] = {0.0, 0.0, 0.0};

  // ============================================================
  //  5. MIT 参数 & 关节限位写入
  // ============================================================
  ArmMotion::Joint6Dof arm_stiffness{};
  ArmMotion::Joint6Dof arm_damping{};
  ArmMotion::Joint6Dof joint_min{};
  ArmMotion::Joint6Dof joint_max{};

  int count = 0;
  joint_names_.clear();
  for (const auto& joint : robot_cfg["joints"]) {
    arm_stiffness[count] = joint["MIT"]["stiffness"].as<double>();
    arm_damping[count] = joint["MIT"]["damping"].as<double>();
    joint_min[count] = limits_lower[count];
    joint_max[count] = limits_upper[count];
    joint_names_.push_back(joint["name"].as<std::string>());
    count++;
  }
  arm_controller_->SetJointLimits(joint_min, joint_max);
  arm_controller_->SetMotorParams(arm_stiffness, arm_damping);

  // ============================================================
  //  Dynamics 配置（可选：从 YAML 文件或内联 node 读取）
  // ============================================================
  try {
    std::vector<Dynamics::Link> links;
    Dynamics::Link base{};
    links.push_back(base);
    std::vector<std::string> link_names;  // 与 links[1..N] 一一对应（links[0] 为基座占位）
    std::vector<double> gravity;
    if (robot_cfg["dynamics"]) {
      if (!robot_cfg["dynamics"].IsScalar()) {
        ARM_LOG_WARN("请输入正确的 Dynamics 配置文件名");
      } else {
        std::string dyn_file = "./cfg/" + robot_cfg["dynamics"].as<std::string>();
        YAML::Node dyn_root = YAML::LoadFile(dyn_file)["dynamics"];

        if (dyn_root["gravity"]) {
          gravity = dyn_root["gravity"].as<std::vector<double>>();
        }

        if (dyn_root["links"]) {
          for (const auto& l : dyn_root["links"]) {
            Dynamics::Link link{};
            link.mass = l["mass"].as<double>();
            auto com = l["com"].as<std::vector<double>>();
            if (com.size() != 3) {
              ARM_LOG_ERROR("Dynamics 配置中 link 的 com 必须为 3 元素向量");
              continue;
            }
            for (int i = 0; i < 3; ++i) {
              link.com[i] = com[i];
            }

            auto inertia = l["inertia"];
            double ixx = inertia["ixx"] ? inertia["ixx"].as<double>() : 0.0;
            double iyy = inertia["iyy"] ? inertia["iyy"].as<double>() : 0.0;
            double izz = inertia["izz"] ? inertia["izz"].as<double>() : 0.0;
            double ixy = inertia["ixy"] ? inertia["ixy"].as<double>() : 0.0;
            double ixz = inertia["ixz"] ? inertia["ixz"].as<double>() : 0.0;
            double iyz = inertia["iyz"] ? inertia["iyz"].as<double>() : 0.0;
            // 填充为行优先的 3x3 矩阵
            link.inertia[0] = ixx;
            link.inertia[1] = ixy;
            link.inertia[2] = ixz;
            link.inertia[3] = ixy;
            link.inertia[4] = iyy;
            link.inertia[5] = iyz;
            link.inertia[6] = ixz;
            link.inertia[7] = iyz;
            link.inertia[8] = izz;
            links.push_back(link);
            link_names.push_back(l["name"] ? l["name"].as<std::string>() : std::string());
          }
        }

        // ---- 末端工具（tool）惯量融合 ----
        if (dyn_root["tool"]) {
          auto tool = dyn_root["tool"];
          const std::string attach_to = tool["attach_to"].as<std::string>();
          const double tool_mass = tool["mass"].as<double>();
          const double tool_com[3] = {tool["com"][0].as<double>(),
                                      tool["com"][1].as<double>(),
                                      tool["com"][2].as<double>()};
          auto ti = tool["inertia"];
          const double tool_inertia[9] = {
              ti["ixx"].as<double>(), ti["ixy"].as<double>(), ti["ixz"].as<double>(),
              ti["ixy"].as<double>(), ti["iyy"].as<double>(), ti["iyz"].as<double>(),
              ti["ixz"].as<double>(), ti["iyz"].as<double>(), ti["izz"].as<double>()};

          int target = -1;
          for (size_t i = 0; i < link_names.size(); ++i) {
            if (link_names[i] == attach_to) {
              target = static_cast<int>(i) + 1;  // links[0] 为基座占位
              break;
            }
          }

          if (target < 0) {
            ARM_LOG_WARN("Dynamics: tool.attach_to='{}' 未在 links 中找到，未融合", attach_to);
          } else {
            const double m_before = links[target].mass;
            const double c_before[3] = {links[target].com[0], links[target].com[1],
                                        links[target].com[2]};
            FuseToolIntoLink(links[target], tool_mass, tool_com, tool_inertia);
            ARM_LOG_INFO(
                "Dynamics: 已融合末端工具 → {}: mass {:.5f} → {:.5f} kg, "
                "com [{:.5f}, {:.5f}, {:.5f}] → [{:.5f}, {:.5f}, {:.5f}] m",
                attach_to, m_before, links[target].mass, c_before[0], c_before[1],
                c_before[2], links[target].com[0], links[target].com[1],
                links[target].com[2]);
          }
        }

        if (!links.empty() || !gravity.empty()) {
          arm_controller_->SetDynamicsModel(links, gravity);
          ARM_LOG_INFO("Dynamics 初始化: links={}, gravity=[{:.3f}, {:.3f}, {:.3f}]",
                       links.size(), gravity.size() > 0 ? gravity[0] : 0.0, gravity.size() > 1 ? gravity[1] : 0.0, gravity.size() > 2 ? gravity[2] : 0.0);
        }
      }
    }
  } catch (const std::exception& e) {
    ARM_LOG_ERROR("加载 dynamics 配置失败: {}", e.what());
    return;
  }

  // ============================================================
  //  Tool 配置
  // ============================================================
  try {
    YAML::Node tool_node;
    if (robot_cfg["tool"]) {
      tool_node = robot_cfg["tool"];
    }

    if (tool_node) {
      if (tool_node["xyz"] && tool_node["rpy"]) {
        auto xyz = tool_node["xyz"].as<std::vector<double>>();
        auto rpy_deg = tool_node["rpy"].as<std::vector<double>>();
        if (xyz.size() == 3 && rpy_deg.size() == 3) {
          const double kDeg2Rad = M_PI / 180.0;
          double roll = rpy_deg[0] * kDeg2Rad;   // Rx
          double pitch = rpy_deg[1] * kDeg2Rad;  // Ry
          double yaw = rpy_deg[2] * kDeg2Rad;    // Rz

          Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
          Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
          Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
          Eigen::Quaterniond q = rz * ry * rx;  // ZYX sequence

          ArmMotion::Pose6Dof tool_pose{};
          tool_pose.x = xyz[0];
          tool_pose.y = xyz[1];
          tool_pose.z = xyz[2];
          tool_pose.quat = ArmMotion::Quaternion(q.w(), q.x(), q.y(), q.z());

          arm_controller_->SetToolPose(tool_pose);

          // 缓存配置，延迟到初始化打印中统一输出
          tool_configured = true;
          tool_frame = tool_node["frame"] ? tool_node["frame"].as<std::string>() : std::string("tcp");
          tool_xyz_cache[0] = xyz[0]; tool_xyz_cache[1] = xyz[1]; tool_xyz_cache[2] = xyz[2];
          tool_rpy_cache_deg[0] = rpy_deg[0]; tool_rpy_cache_deg[1] = rpy_deg[1]; tool_rpy_cache_deg[2] = rpy_deg[2];
        } else {
          ARM_LOG_WARN("tool.xyz 或 tool.rpy 长度不为 3，忽略 Tool 设置");
        }
      }
    }
  } catch (const std::exception& e) {
    ARM_LOG_ERROR("解析 tool 配置失败: {}", e.what());
  }

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
    ARM_LOG_WARN("轨迹模式 '{}' 不在可选范围内 ['quintic', 'cubic', 'lspb']，默认使用 quintic", traj_mode_str);
  }

  // ============================================================
  //  7. 采样频率
  // ============================================================
  double sample_frequency = robot_cfg["sample_frequency"].as<double>();
  arm_controller_->SetSampleFrequency(sample_frequency);
  arm_controller_->StartSampleThread();
  // ============================================================
  //  8. 夹爪控制器 — 仅在配置中存在时创建
  // ============================================================
  double gripper_stiffness = 0.0, gripper_damping = 0.0, gripper_max_range = 0.0;
  if (robot_cfg["gripper"]) {
    auto gripper_cfg = robot_cfg["gripper"];
    gripper_stiffness = gripper_cfg["MIT"]["stiffness"].as<double>();
    gripper_damping = gripper_cfg["MIT"]["damping"].as<double>();
    gripper_max_range = gripper_cfg["max_range"].as<double>();
    joint_names_.push_back(gripper_cfg["name"].as<std::string>());

    gripper_controller_ = std::make_shared<GripperController>(gripper_stiffness, gripper_damping);
    gripper_controller_->SetMaxRange(gripper_max_range);
    gripper_controller_->SetSampleFrequency(sample_frequency);
  }

  // ============================================================
  //  9. 打印初始化参数
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
                   arm_stiffness[i], arm_damping[i]);
    }

    // 轨迹 & 夹爪
    ARM_LOG_INFO("{}", kSepM);
    ARM_LOG_INFO("  Trajectory: {}", traj_mode_str);
    if (gripper_controller_) {
      ARM_LOG_INFO("  Gripper:    range={:.3f}, stiffness={:.1f}, damping={:.1f}",
                   gripper_max_range, gripper_stiffness, gripper_damping);
    } else {
      ARM_LOG_INFO("  Gripper:    (未配置)");
    }
    // Tool 信息（如果配置则打印）
    if (tool_configured) {
      ARM_LOG_INFO("  TCP:       frame={}, xyz=[{:.3f}, {:.3f}, {:.3f}], rpy(deg)=[{:.3f}, {:.3f}, {:.3f}]",
                   tool_frame,
                   tool_xyz_cache[0], tool_xyz_cache[1], tool_xyz_cache[2],
                   tool_rpy_cache_deg[0], tool_rpy_cache_deg[1], tool_rpy_cache_deg[2]);
    } else {
      ARM_LOG_INFO("  Tool:       (未配置)");
    }
    ARM_LOG_INFO("{}", kSepH);
  }
}
