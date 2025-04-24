import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Load the trajectory
df = pd.read_csv("min_jerk_traj.csv")

# Plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.plot(df['x'], df['y'], df['z'], label="Min-Jerk Trajectory")
ax.scatter(df['x'].iloc[0], df['y'].iloc[0], df['z'].iloc[0], color='green', label='Start')
ax.scatter(df['x'].iloc[-1], df['y'].iloc[-1], df['z'].iloc[-1], color='red', label='End')

dt = 0.01
alt = 3
x0 = 3.2
y0 = 0.0
theta0 = 0
xf = 10.0
yf = -20.0
control_point = [(x0, y0, 0),  (35.0, 0.0, alt), (45.0, 0.0, alt), (45.0, 25.0, alt), (-8.0, 25.0, alt), (-10.0, 1.0,alt), (5, -15.0, 2.0), (xf, yf,0.1)]

control_point = np.array(control_point)
ax.scatter(control_point[:, 0], control_point[:, 1], control_point[:, 2], color='r', label="Reference Waypoints")
for i, point in enumerate(control_point):
    ax.text(point[0], point[1], point[2], f'({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f})', 
            color='black', fontsize=10)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()
plt.title("Minimum Jerk Trajectory")
plt.savefig("min_jerk.jpg")
plt.show()
