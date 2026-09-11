#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "actuator_manager.h"
#include "arm_logger.h"

namespace {

constexpr double kZeroPosTol = 0.001;  // rad，与控制层 CalibrateZero 的验证容差一致
constexpr int kArmJointNum = 6;

std::atomic<bool> g_running{true};

void SignalHandler(int) { g_running = false; }

// ---------------------------------------------------------------------------
// 终端输出辅助（中文字符按显示宽度 2 列对齐）
// ---------------------------------------------------------------------------
size_t DisplayWidth(const std::string& text) {
  size_t width = 0;
  for (size_t i = 0; i < text.size();) {
    const unsigned char c = static_cast<unsigned char>(text[i]);
    if (c < 0x80) {
      width += 1;
      i += 1;
    } else if ((c >> 5) == 0x6) {
      width += 1;
      i += 2;
    } else if ((c >> 4) == 0xE) {
      width += 2;  // CJK 占两列
      i += 3;
    } else {
      width += 2;
      i += 4;
    }
  }
  return width;
}

std::string Pad(const std::string& text, size_t width) {
  const size_t cur = DisplayWidth(text);
  return text + std::string(cur < width ? width - cur : 0, ' ');
}

void PrintLine() { fmt::print("{:-<78}\n", ""); }

void PrintTitle(const std::string& title) {
  PrintLine();
  fmt::print("{}\n", title);
  PrintLine();
}

// ---------------------------------------------------------------------------
// 安全检查：确认主程序未运行
// ---------------------------------------------------------------------------
bool CheckMainProgramStopped(bool force) {
  const bool running = (std::system("pgrep -x aimrt_main > /dev/null 2>&1") == 0);
  if (!running) {
    fmt::print("[安全检查] 未检测到 aimrt_main 进程，可以标零\n");
    return true;
  }

  fmt::print("\n{}[安全检查] 检测到 aimrt_main 正在运行！{}\n", Ansi::kRed, Ansi::kReset);
  fmt::print("  标零会改变关节零点；若控制栈同时在运行，零点跳变会导致关节猛动。\n");
  fmt::print("  请先停止控制栈后再运行本工具。\n");
  if (!force) {
    return false;
  }
  fmt::print("\n  已指定 --force，仍要继续？输入 yes 确认：");
  std::string answer;
  std::cin >> answer;
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return answer == "yes";
}

// ---------------------------------------------------------------------------
// 状态与结果的表格输出
// ---------------------------------------------------------------------------
void PrintArmState(const std::string& title, const ArmCtrlBase::JointState& state,
                   const std::vector<std::string>& names) {
  fmt::print("\n>>> {}\n", title);
  fmt::print("{}{}{}{}{}\n", Pad("关节", 6), Pad("名称", 22), Pad("位置(rad)", 16),
             Pad("速度(rad/s)", 16), Pad("力矩(N·m)", 14));
  PrintLine();
  for (int i = 0; i < kArmJointNum; ++i) {
    const std::string name = (i < static_cast<int>(names.size())) ? names[i] : "-";
    fmt::print("{}{}{:>16.5f}{:>16.5f}{:>14.5f}\n", Pad(std::to_string(i), 6), Pad(name, 22),
               state.position[i], state.velocity[i], state.effort[i]);
  }
}

void PrintArmResult(uint32_t verified_mask, const ArmCtrlBase::JointState& state,
                    const std::vector<std::string>& names) {
  int ok_count = 0;
  fmt::print("\n>>> 标零结果\n");
  fmt::print("{}{}{}{}{}\n", Pad("关节", 6), Pad("名称", 22), Pad("标零", 10),
             Pad("当前读数(rad)", 16), Pad("偏差判定", 12));
  PrintLine();
  for (int i = 0; i < kArmJointNum; ++i) {
    const bool ok = (verified_mask & (1u << i)) != 0;
    ok_count += ok ? 1 : 0;
    const std::string name = (i < static_cast<int>(names.size())) ? names[i] : "-";
    const double pos = state.position[i];
    const bool within = std::fabs(pos) <= kZeroPosTol;

    fmt::print("{}{}", Pad(std::to_string(i), 6), Pad(name, 22));
    if (ok) {
      fmt::print("{}{}{}", Ansi::kGreen, Pad("成功", 10), Ansi::kReset);
    } else {
      fmt::print("{}{}{}", Ansi::kRed, Pad("失败", 10), Ansi::kReset);
    }
    fmt::print("{:>16.5f}{}", pos, Pad("", 4));
    if (within) {
      fmt::print("{}{}{}", Ansi::kGreen, "≈0 通过", Ansi::kReset);
    } else {
      fmt::print("{}{}{}", Ansi::kYellow, "超出容差", Ansi::kReset);
    }
    fmt::print("\n");
  }
  PrintLine();

  if (ok_count == kArmJointNum) {
    fmt::print("{}本体标零：全部 {} 个关节成功{}\n", Ansi::kGreen, kArmJointNum, Ansi::kReset);
  } else {
    fmt::print("{}本体标零：成功 {}/{}，失败关节：", Ansi::kRed, ok_count, kArmJointNum);
    for (int i = 0; i < kArmJointNum; ++i) {
      if ((verified_mask & (1u << i)) == 0) {
        fmt::print("{} ", i);
      }
    }
    fmt::print("{}\n", Ansi::kReset);
  }
}

void PrintGripperResult(bool ok, const GripperCtrlBase::GripperState& state) {
  fmt::print("\n>>> 标零结果\n");
  fmt::print("{}{}{}\n", Pad("部件", 12), Pad("标零", 10), Pad("当前读数(rad)", 16));
  PrintLine();
  fmt::print("{}", Pad("夹爪", 12));
  if (ok) {
    fmt::print("{}{}{}", Ansi::kGreen, Pad("成功", 10), Ansi::kReset);
  } else {
    fmt::print("{}{}{}", Ansi::kRed, Pad("失败", 10), Ansi::kReset);
  }
  fmt::print("{:>16.5f}\n", state.position);
  PrintLine();
}

// ---------------------------------------------------------------------------
// 标零动作
// ---------------------------------------------------------------------------
void DoArmZero(const std::shared_ptr<ArmCtrlBase>& arm, const std::vector<std::string>& names) {
  ArmCtrlBase::JointState before;
  arm->GetJointState(before);
  PrintArmState("标零前位置快照", before, names);

  fmt::print("\n>>> 请确认机械臂已按机械基准（工装/刻线）摆到零位姿态，按回车开始标零...");
  std::cin.get();
  if (!g_running) {
    return;
  }

  fmt::print(">>> 正在标零本体（发送六帧置零指令 + 逐关节回读验证）...\n");
  const uint32_t verified_mask = arm->CalibrateZero();

  ArmCtrlBase::JointState after;
  arm->GetJointState(after);
  PrintArmResult(verified_mask, after, names);
}

void DoGripperZero(const std::shared_ptr<GripperCtrlBase>& gripper) {
  fmt::print("\n>>> 请确认夹爪处于期望的零位姿态，按回车开始标零...");
  std::cin.get();
  if (!g_running) {
    return;
  }

  fmt::print(">>> 正在标零夹爪...\n");
  const bool ok = gripper->CalibrateZero();

  GripperCtrlBase::GripperState after;
  gripper->ReceiveState(after);
  PrintGripperResult(ok, after);
}

void PrintMenu(bool has_gripper) {
  fmt::print("\n可用操作：\n");
  fmt::print("  1) 标零本体（六关节）\n");
  if (has_gripper) {
    fmt::print("  2) 标零夹爪\n");
    fmt::print("  3) 都标零（先本体，后夹爪）\n");
  }
  fmt::print("  0) 退出\n");
  fmt::print("请输入选项：");
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGINT, SignalHandler);

  std::string cfg_file = "actuator_module/geniarm_actuator_cfg.yaml";
  bool force = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--force") {
      force = true;
    } else {
      cfg_file = arg;
    }
  }

  PrintTitle("Geniarm 关节标零工具");
  fmt::print("执行器配置：{}（相对 ./cfg/）\n", cfg_file);

  if (!CheckMainProgramStopped(force)) {
    return 1;
  }

  const std::string cfg_path = "./cfg/" + cfg_file;
  if (std::system(("test -f " + cfg_path).c_str()) != 0) {
    fmt::print("{}错误：找不到配置文件 {}，请在包含 cfg/ 的目录下运行{}\n",
               Ansi::kRed, cfg_path, Ansi::kReset);
    return 1;
  }

  std::shared_ptr<ActuatorManager> actuator_manager;
  try {
    actuator_manager = std::make_shared<ActuatorManager>(cfg_file);
  } catch (const std::exception& e) {
    fmt::print("{}错误：创建 ActuatorManager 失败：{}{}\n", Ansi::kRed, e.what(), Ansi::kReset);
    return 1;
  }

  auto arm = actuator_manager->GetArmActuator();
  auto gripper = actuator_manager->GetGripperActuator();  // 配置里没有 gripper 段时为空
  const bool has_gripper = (gripper != nullptr);

  std::vector<std::string> joint_names;
  actuator_manager->GetJointNames(joint_names);

  fmt::print("夹爪：{}\n", has_gripper ? "已配置" : "未配置（配置文件中无 gripper 段）");
  fmt::print("电机保持失能状态，可手动反驱摆位\n");

  while (g_running) {
    PrintMenu(has_gripper);

    std::string choice;
    if (!(std::cin >> choice)) {
      break;
    }
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

    if (choice == "0") {
      break;
    }

    if (choice == "1") {
      DoArmZero(arm, joint_names);
    } else if (choice == "2" && has_gripper) {
      DoGripperZero(gripper);
    } else if (choice == "3" && has_gripper) {
      DoArmZero(arm, joint_names);
      DoGripperZero(gripper);
    } else {
      fmt::print("{}无效选项{}\n", Ansi::kYellow, Ansi::kReset);
      continue;
    }

    fmt::print("\n{}提示：标零完成后请重启控制栈（aimrt_main），"
               "控制模块会以新零点下的实测位置作为保持目标。{}\n",
               Ansi::kCyan, Ansi::kReset);
  }

  fmt::print("退出标零工具。\n");

  std::signal(SIGINT, SIG_DFL);   // 恢复默认，退出阶段Ctrl+C才有效
  actuator_manager->Shutdown();   // 停CAN采样线程
  return 0;
}
