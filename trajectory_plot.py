import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("trajectory.csv")

plt.figure()
plt.plot(data["t"], data["pos"])
plt.title("Position")
plt.xlabel("Time (s)")
plt.ylabel("Position")
plt.grid()

plt.figure()
plt.plot(data["t"], data["vel"])
plt.title("Velocity")
plt.xlabel("Time (s)")
plt.ylabel("Velocity")
plt.grid()

plt.figure()
plt.plot(data["t"], data["acc"])
plt.title("Acceleration")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration")
plt.grid()

plt.show()
