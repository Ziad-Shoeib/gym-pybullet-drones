import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline
from mpl_toolkits.mplot3d import Axes3D

# Define waypoints
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

# Time parameter (uniform for now)
t = np.linspace(0, 1, len(waypoints))

# Approximate minimum snap using quintic spline interpolation
spline_x = make_interp_spline(t, waypoints[:, 0], k=5)  # k=5 → quintic
spline_y = make_interp_spline(t, waypoints[:, 1], k=5)
spline_z = make_interp_spline(t, waypoints[:, 2], k=5)

# Evaluate spline at high resolution
t_dense = np.linspace(0, 1, 300)
x_traj = spline_x(t_dense)
y_traj = spline_y(t_dense)
z_traj = spline_z(t_dense)

# Plot the trajectory
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_traj, y_traj, z_traj, label='Minimum Snap Approximation', linewidth=2)
ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
           color='red', label='Waypoints', s=50)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Quintic Spline Approximation of Minimum Snap Trajectory')
ax.legend()
plt.tight_layout()
plt.show()
