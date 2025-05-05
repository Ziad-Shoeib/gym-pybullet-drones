import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from mpl_toolkits.mplot3d import Axes3D

# Define the waypoints
waypoints = np.array([
    [0.0, 0.0, 0.5],
    [0.5, 0.5, 1.0],
    [1.0, 1.25, 1.5],
    [1.5, 1.25, 1.5],
    [2.0, 1.25, 1.5],
    [2.5, 1.25, 1.3],
    [3.0, 1.25, 1.0],
    [3.0, 1.25, 0.5],
    [3.0, 1.25, 0.1],
])

# Create a normalized time vector
t = np.linspace(0, 1, len(waypoints))

# Fit cubic splines through the waypoints for each coordinate
cs_x = CubicSpline(t, waypoints[:, 0])
cs_y = CubicSpline(t, waypoints[:, 1])
cs_z = CubicSpline(t, waypoints[:, 2])

# Evaluate the spline at high resolution
t_dense = np.linspace(0, 1, 300)
x_cubic = cs_x(t_dense)
y_cubic = cs_y(t_dense)
z_cubic = cs_z(t_dense)

# Plot the interpolated trajectory and waypoints
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_cubic, y_cubic, z_cubic, label='Cubic Spline Trajectory', linewidth=2)
ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
           color='red', label='Waypoints', s=50)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Smooth Trajectory using Cubic Splines')
ax.legend()
plt.tight_layout()
plt.show()
