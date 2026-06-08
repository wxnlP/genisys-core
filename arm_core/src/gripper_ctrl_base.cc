#include "gripper_ctrl_base.h"

#include "lspb_time_scaler.h"
#include "arm_logger.h"
#include <chrono>
#include <cmath>
#include <thread>
#include <condition_variable>

bool GripperCtrlBase::SetPosition(double position, double velocity,
                                  double accel, double effort) {
  const double max_range = max_range_;

  // 读取当前物理位置，转归一化
  double current_pos{};
  double current_vel{};
  double current_effort{};
  if (!ReceiveState(current_pos, current_vel, current_effort)) {
    return false;
  }

  ARM_LOG_INFO("Current State: {} {} {}", current_pos, current_vel, current_effort);

  double target_pos = position * max_range;

  // 已在目标位置
  double distance = target_pos - current_pos;
  if (std::abs(distance) < 1e-6) {
    return true;
  }

  // ── 物理值 → 归一化
  const double abs_dist = std::abs(distance);
  if (velocity <= 0.0 || accel <= 0.0 || abs_dist < 1e-9) {
    return false;
  }

  LSPBTimeScaler scaler;
  TimeScalerBase::PlanConfig config;
  config.v_max = velocity / abs_dist;
  config.a_max = accel / abs_dist;
  if (!scaler.Plan(config)) {
    return false;
  }

  const double duration = scaler.GetDuration();
  double t = 0.0;
  while (t <= duration + 1e-7) {
    if (t > duration) {
      t = duration;
    }

    TimeScalerBase::State state;
    if (!scaler.Evaluate(t, state)) {
      return false;
    }

    // 归一化 → 物理值
    const double pos_phys = current_pos + state.s * distance;
    const double vel_phys = state.sd * distance;
    // const double accel_phys = state.sdd * distance;

    if (!SendCommand(pos_phys, vel_phys, effort, motor_stiffness_, motor_damping_)) {
      return false;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(kDefaultDt));
    t += kDefaultDt;
  }

  return true;
}

bool GripperCtrlBase::SetTorque(double effort, double ramp_rate) {
  double current_pos{}, current_vel{}, current_effort{};
  if (!ReceiveState(current_pos, current_vel, current_effort)) {
    ARM_LOG_ERROR("SetTorque: ReceiveState failed");
    return false;
  }

  ARM_LOG_INFO("SetTorque: current_effort={:.3f}Nm, target={:.3f}Nm, ramp={:.1f}Nm/s",
               current_effort, effort, ramp_rate);
  
  // 匀速增加力矩
  const double delta = effort - current_effort;
  const double duration = std::abs(delta) / ramp_rate;
  double t = 0.0;

  // Kd=0.5 阻尼刹车
  constexpr double kKd = 0.5;

  // 线程管理
  std::atomic<bool> pos_limited{false};
  std::condition_variable cv;
  std::mutex cv_mutex;

  std::jthread pos_limit_thread([this, &cv, &pos_limited, &effort] (std::stop_token stoken)  {
    // 位置监控
    while (!stoken.stop_requested()) {
      double now_pos{}, now_vel{}, now_effort{};
      ReceiveState(now_pos, now_vel, now_effort);
      if (std::abs(now_pos) > std::abs(max_range_)+0.5) {
        pos_limited = true;
        cv.notify_all();
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    // 卸力
    if (pos_limited && !stoken.stop_requested()) {
      ARM_LOG_WARN("Triggered pos limit");
      constexpr double kBrakeRate = 10.0;  // Nm/s
      const double brake_dur = std::abs(effort) / kBrakeRate;
      double t = 0.0;

      while (t <= brake_dur + 1e-7) {
        if (t > brake_dur) { t = brake_dur; }
        const double cmd_effort = effort * (1.0 - t / brake_dur);

        SendCommand(0.0, 0.0, cmd_effort, 0.0, kKd);
        std::this_thread::sleep_for(std::chrono::duration<double>(kDefaultDt));
        t += kDefaultDt;
      }

      SendCommand(0.0, 0.0, 0.0, 0.0, kKd);
      ARM_LOG_INFO("SetTorque: SAFE, torque released");
    }
  });

  while (t <= duration + 1e-7 && (!pos_limited)) {
    if (t > duration) { t = duration; }

    const double alpha = (duration > 1e-9) ? (t / duration) : 1.0;
    const double cmd_effort = current_effort + delta * alpha;

    if (!SendCommand(0.0, 0.0, cmd_effort, 0.0, kKd)) {
      ARM_LOG_ERROR("SetTorque: SendCommand failed at t={:.3f}s, "
                    "effort={:.3f}Nm",
                    t, cmd_effort);
      return false;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(kDefaultDt));
    t += kDefaultDt;
  }

  if (!pos_limited) {
    ARM_LOG_INFO("SetTorque: Target torque reached. Waiting for pos limit (max 3s)...");

    std::unique_lock<std::mutex> lk(cv_mutex);
    // 使用条件变量的 wait_for，在 3 秒内等待 pos_limited 变成 true
    bool cv_ret = cv.wait_for(lk, std::chrono::seconds(3), [&pos_limited]() { 
      return pos_limited.load(); 
    });

    if (!cv_ret) {
      // 情况 2：卡住了，超过3秒依然没有触发限位
      ARM_LOG_ERROR("SetTorque: Timeout! Position limit not reached within 3s. Exit with error.");
      
      // 停止硬件输出（防止继续用力导致电机过热或其他危险）
      SendCommand(0.0, 0.0, 0.0, 0.0, kKd); 
      
      // 【关键】由于是异常退出，不需要子线程去走卸力流程，直接强制叫停子线程
      pos_limit_thread.request_stop(); 
      pos_limit_thread.join(); // 等待线程安全消亡
      return false;            // 触发异常并退出
    }
  }

  // 情况 1：不管是斜坡中途触发的，还是在3秒等待期间触发的限位，都会走到这里
  ARM_LOG_INFO("SetTorque: Pos limit detected successfully. Waiting for brake release...");
  
  // 此时子线程正在执行“刹车卸力循环”，主函数在此 join 挂起等待
  // 当子线程卸力完毕退出后，join() 结束，主函数与子线程实现完美同步退出
  pos_limit_thread.join(); 

  ARM_LOG_INFO("SetTorque: Task completed successfully.");

  return true;
}

bool GripperCtrlBase::GetPosition(double& position, double& effort) {
  double position_phys = 0.0;
  double velocity_phys = 0.0;
  if (!ReceiveState(position_phys, velocity_phys, effort)) {
    return false;
  }
  position = position_phys / max_range_;
  return true;
}
