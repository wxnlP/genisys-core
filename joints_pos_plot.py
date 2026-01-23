import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("joint_pos.csv")

t = df["t"].values
plt.figure(figsize=(10,6))
for i in range(6):
    plt.plot(t, df[f"q{i+1}"], label=f"q{i+1}")

plt.xlabel("Time [s]")
plt.ylabel("Joint Position [rad]")
plt.legend()
plt.grid()
plt.title("Joint Space Positions")
plt.show()
