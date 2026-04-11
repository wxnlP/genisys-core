import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("cartesian.csv")

fig = plt.figure(figsize=(14, 12))
grid = fig.add_gridspec(4, 2)

ax_path = fig.add_subplot(grid[0, :], projection='3d')
ax_pos = fig.add_subplot(grid[1, 0])
ax_quat = fig.add_subplot(grid[1, 1])
ax_linear_vel = fig.add_subplot(grid[2, 0])
ax_angular_vel = fig.add_subplot(grid[2, 1])
ax_linear_acc = fig.add_subplot(grid[3, 0])
ax_angular_acc = fig.add_subplot(grid[3, 1])

t = df["t"].values

ax_path.plot(df["x"], df["y"], df["z"], label="Cartesian Path")
ax_path.scatter(df["x"].iloc[0], df["y"].iloc[0], df["z"].iloc[0], c='g', label="Start")
ax_path.scatter(df["x"].iloc[-1], df["y"].iloc[-1], df["z"].iloc[-1], c='r', label="End")

for column in ["x", "y", "z"]:
	ax_pos.plot(t, df[column], label=column)

for column in ["qw", "qx", "qy", "qz"]:
	ax_quat.plot(t, df[column], label=column)

for column in ["vx", "vy", "vz"]:
	ax_linear_vel.plot(t, df[column], label=column)

for column in ["wx", "wy", "wz"]:
	ax_angular_vel.plot(t, df[column], label=column)

for column in ["ax", "ay", "az"]:
	ax_linear_acc.plot(t, df[column], label=column)

for column in ["alphax", "alphay", "alphaz"]:
	ax_angular_acc.plot(t, df[column], label=column)

ax_path.set_xlabel("X")
ax_path.set_ylabel("Y")
ax_path.set_zlabel("Z")
ax_path.legend()
ax_path.set_title("Cartesian Trajectory")

ax_pos.set_title("Position")
ax_pos.set_ylabel("Position")
ax_pos.grid(True)
ax_pos.legend()

ax_quat.set_title("Quaternion")
ax_quat.grid(True)
ax_quat.legend()

ax_linear_vel.set_title("Linear Velocity")
ax_linear_vel.set_ylabel("Velocity")
ax_linear_vel.grid(True)
ax_linear_vel.legend()

ax_angular_vel.set_title("Angular Velocity")
ax_angular_vel.grid(True)
ax_angular_vel.legend()

ax_linear_acc.set_title("Linear Acceleration")
ax_linear_acc.set_xlabel("Time [s]")
ax_linear_acc.set_ylabel("Accel")
ax_linear_acc.grid(True)
ax_linear_acc.legend()

ax_angular_acc.set_title("Angular Acceleration")
ax_angular_acc.set_xlabel("Time [s]")
ax_angular_acc.grid(True)
ax_angular_acc.legend()

plt.tight_layout()
plt.show()
