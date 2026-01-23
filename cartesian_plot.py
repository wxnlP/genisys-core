import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

df = pd.read_csv("cartesian.csv")

fig = plt.figure(figsize=(7,7))
ax = fig.add_subplot(111, projection='3d')

ax.plot(df["x"], df["y"], df["z"], label="Cartesian Path")
ax.scatter(df["x"].iloc[0],  df["y"].iloc[0],  df["z"].iloc[0], c='g', label="Start")
ax.scatter(df["x"].iloc[-1], df["y"].iloc[-1], df["z"].iloc[-1], c='r', label="End")

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()
ax.set_title("Cartesian XYZ Trajectory")
plt.show()
