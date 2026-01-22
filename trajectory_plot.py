import pandas as pd
import matplotlib.pyplot as plt

# 设置 STIX 字体（类 LaTeX），解决找不到 Times New Roman 的问题
plt.rcParams.update({
    "font.family": "STIXGeneral",
    "mathtext.fontset": "stix",
    "font.size": 11,
    "axes.unicode_minus": False
})

# 模拟数据 (供你测试)
traj = pd.read_csv("trajectory.csv")
via  = pd.read_csv("via_points.csv")

# 颜色定义
COLOR_LINE1 = '#1f77b4'  # 专业蓝
COLOR_LINE2 = "#de821a"  # 专业蓝
COLOR_LINE3 = "#1cad53"  # 专业蓝
COLOR_VIA = '#d62728'   # 稳重红

# 绘图比例调整为更协调的 (8, 7)
fig, axs = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
ax_pos, ax_vel, ax_acc = axs

# --- 1. Position ---
ax_pos.plot(traj["t"], traj["pos"], color=COLOR_LINE1, lw=2, label='Trajectory')
# ax_pos.scatter(via["t_via"], via["theta_via"], 
#                marker='o', s=35, fc='white', ec=COLOR_VIA, lw=1.2, zorder=5, label='ViaPoints')
ax_pos.set_ylabel('Position (rad)')
# 参考图片，将图例放在绘图区内部右下角，不遮挡起始曲线
ax_pos.legend(loc='lower right', frameon=True, edgecolor='none', facecolor='white', framealpha=0.8)

# --- 2. Velocity ---
ax_vel.plot(traj["t"], traj["vel"], color=COLOR_LINE2, lw=1.8)
ax_vel.set_ylabel('Velocity (rad/s)')

# --- 3. Acceleration ---
ax_acc.plot(traj["t"], traj["acc"], color=COLOR_LINE3, lw=1.8)
ax_acc.set_ylabel('Accel (rad/s$^2$)')
ax_acc.set_xlabel('Time (s)')

# --- 统一细节处理 ---
for ax in axs:
    # 设置刻度向内，且四周都有刻度线
    ax.tick_params(direction='in', top=True, right=True, which='both')
    # 浅灰色细网格
    ax.grid(True, ls=':', lw=0.5, color='#d1d1d1')
    
    # 垂直时间节点辅助线（参考图片 1/2）
    for t_v in via["t_via"]:
        ax.axvline(x=t_v, color='gray', ls='--', lw=0.8, alpha=0.3, zorder=1)

# 关键：消除子图间的纵向间距，让它们看起来像一个整体
plt.subplots_adjust(hspace=0)

# 自动处理边缘白边
plt.tight_layout()
# 再次微调 hspace 以确保紧凑
plt.subplots_adjust(hspace=0.05)

plt.savefig("trajectory_optimized.png", dpi=300, bbox_inches='tight')
plt.show()