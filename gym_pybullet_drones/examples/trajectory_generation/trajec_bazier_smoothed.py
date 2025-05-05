import numpy as np
from scipy.special import comb
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

# Original waypoints (control points)
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

# ✅ 1. Smooth the waypoints
waypoints_smoothed = savgol_filter(waypoints, window_length=5, polyorder=2, axis=0)

# ✅ 2. Bezier curve generation
def bezier_curve(control_points, num_points=300):
    n = len(control_points) - 1
    t = np.linspace(0, 1, num_points)
    curve = np.zeros((num_points, 3))
    for i in range(n + 1):
        bernstein_poly = comb(n, i) * (t ** i) * ((1 - t) ** (n - i))
        curve += np.outer(bernstein_poly, control_points[i])
    return curve

# ✅ 3. Derivative of Bezier curve
def bezier_derivative(control_points, order=1, num_points=300):
    cp = control_points.copy()
    n = len(cp) - 1
    for _ in range(order):
        cp = np.array([n * (cp[i+1] - cp[i]) for i in range(len(cp)-1)])
        n -= 1
    return bezier_curve(cp, num_points)

# ✅ 4. Arc-length reparameterization (for visualization or time warping)
def arc_length_parametrize(curve):
    deltas = np.diff(curve, axis=0)
    distances = np.linalg.norm(deltas, axis=1)
    arc_lengths = np.insert(np.cumsum(distances), 0, 0)
    arc_lengths /= arc_lengths[-1]  # Normalize to [0, 1]
    return arc_lengths

# Generate trajectory
position = bezier_curve(waypoints_smoothed)
velocity = bezier_derivative(waypoints_smoothed, order=1)
acceleration = bezier_derivative(waypoints_smoothed, order=2)
arc_param = arc_length_parametrize(position)

# Save position only
df_pos = pd.DataFrame(position, columns=["x", "y", "z"])
df_pos.to_csv("bezier_position.csv", index=False)

# Save velocity only
df_vel = pd.DataFrame(velocity, columns=["vx", "vy", "vz"])
df_vel.to_csv("bezier_velocity.csv", index=False)

# ✅ 5. Plot 3D trajectory + velocity vectors
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(position[:, 0], position[:, 1], position[:, 2], label='Bezier Trajectory', linewidth=2)
ax.quiver(position[::20, 0], position[::20, 1], position[::20, 2],
          velocity[::20, 0], velocity[::20, 1], velocity[::20, 2],
          length=0.2, normalize=True, color='blue', label='Velocity Vectors')
ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], color='red', label='Original Waypoints', s=50)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Bezier Trajectory with Velocity and Smoothed Waypoints')
ax.legend()
plt.tight_layout()
plt.show()

# Plot the velocity trajectory in 3D
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(velocity[:, 0], velocity[:, 1], velocity[:, 2], color='blue', linewidth=2, label='Velocity Trajectory')
ax.set_xlabel('Vx')
ax.set_ylabel('Vy')
ax.set_zlabel('Vz')
ax.set_title('3D Velocity Trajectory from Bezier Curve')
ax.legend()
plt.tight_layout()
plt.show()
