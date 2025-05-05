import numpy as np
from scipy.special import comb
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.signal import savgol_filter

# Define the waypoints (used as control points for the Bezier curve)
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
waypoints_smoothed = savgol_filter(waypoints, window_length=5, polyorder=2, axis=0)
# Function to compute a high-order Bezier curve
def bezier_curve(control_points, num_points=300):
    n = len(control_points) - 1
    t = np.linspace(0, 1, num_points)
    curve = np.zeros((num_points, 3))

    for i in range(n + 1):
        bernstein_poly = comb(n, i) * (t ** i) * ((1 - t) ** (n - i))
        curve += np.outer(bernstein_poly, control_points[i])
    
    return curve

# Generate the curve
bezier_traj = bezier_curve(waypoints)




# Plot the Bezier curve and control points
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(bezier_traj[:, 0], bezier_traj[:, 1], bezier_traj[:, 2], label='High-Order Bezier Trajectory', linewidth=2)
ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2], color='red', label='Control Points (Waypoints)', s=50)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Single High-Order Bezier Curve Through All Waypoints')
ax.legend()
plt.tight_layout()
plt.show()
