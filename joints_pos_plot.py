import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("joint_pos.csv")

t = df["t"].values

fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

for i in range(6):
    axes[0].plot(t, df[f"q{i+1}"], label=f"q{i+1}")
    axes[1].plot(t, df[f"dq{i+1}"], label=f"dq{i+1}")
    axes[2].plot(t, df[f"ddq{i+1}"], label=f"ddq{i+1}")

axes[0].set_ylabel("Position [rad]")
axes[0].set_title("Joint Space Trajectory")
axes[0].grid(True)
axes[0].legend(ncol=3)

axes[1].set_ylabel("Velocity [rad/s]")
axes[1].grid(True)

axes[2].set_xlabel("Time [s]")
axes[2].set_ylabel("Accel [rad/s^2]")
axes[2].grid(True)

plt.tight_layout()
plt.show()
