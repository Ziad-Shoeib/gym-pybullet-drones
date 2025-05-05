import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Define the 3D waypoints
waypoints_3d = np.array([
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

# Function: linear interpolation between waypoints (motion primitives)
def apply_motion_primitive_3d(p0, p1, step_size=0.05):
    vec = p1 - p0
    dist = np.linalg.norm(vec)
    num_steps = max(2, int(dist / step_size))
    interp = np.linspace(0, 1, num_steps).reshape(-1, 1)
    return p0 + interp * vec

# Generate trajectory by chaining motion primitives
primitive_path_3d = []
for i in range(len(waypoints_3d) - 1):
    seg = apply_motion_primitive_3d(waypoints_3d[i], waypoints_3d[i+1])
    primitive_path_3d.append(seg)

primitive_path_3d = np.vstack(primitive_path_3d)

# Plot the 3D motion primitive trajectory
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(primitive_path_3d[:, 0], primitive_path_3d[:, 1], primitive_path_3d[:, 2],
        label="Motion Primitive Path (3D)", linewidth=2)
ax.scatter(waypoints_3d[:, 0], waypoints_3d[:, 1], waypoints_3d[:, 2],
           color='red', label='Waypoints', s=50)
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.set_title("3D Motion Primitive Trajectory Between Waypoints")
ax.view_init(elev=25, azim=135)  # Adjust view for better perspective
ax.legend()
plt.tight_layout()
plt.show()
