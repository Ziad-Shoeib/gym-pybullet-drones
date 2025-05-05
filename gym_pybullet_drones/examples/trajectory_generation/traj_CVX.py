import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
import math

# Waypoints
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

n_points = waypoints.shape[0]
t = np.linspace(0, 1, n_points)
t_dense = np.linspace(0, 1, 300)
degree = 10
lambda_wp = 1000.0

# Variables
coeffs_x = cp.Variable(degree + 1)
coeffs_y = cp.Variable(degree + 1)
coeffs_z = cp.Variable(degree + 1)

# Snap matrix
snap_matrix = np.zeros((len(t_dense), degree + 1))
for i, ti in enumerate(t_dense):
    for j in range(4, degree + 1):
        snap_matrix[i, j] = math.factorial(j) / math.factorial(j - 4) * ti**(j - 4)

# Waypoint fitting matrix
A_wp = np.vander(t, degree + 1, increasing=True)

# Objective
objective = cp.Minimize(
    cp.sum_squares(snap_matrix @ coeffs_x) +
    cp.sum_squares(snap_matrix @ coeffs_y) +
    cp.sum_squares(snap_matrix @ coeffs_z) +
    lambda_wp * cp.sum_squares(A_wp @ coeffs_x - waypoints[:, 0]) +
    lambda_wp * cp.sum_squares(A_wp @ coeffs_y - waypoints[:, 1]) +
    lambda_wp * cp.sum_squares(A_wp @ coeffs_z - waypoints[:, 2])
)

# Solve
problem = cp.Problem(objective)
problem.solve()

# Evaluate and plot
if problem.status == "optimal":
    x_opt = sum([coeffs_x.value[i] * t_dense**i for i in range(degree + 1)])
    y_opt = sum([coeffs_y.value[i] * t_dense**i for i in range(degree + 1)])
    z_opt = sum([coeffs_z.value[i] * t_dense**i for i in range(degree + 1)])

    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(x_opt, y_opt, z_opt, label="Minimum Snap Trajectory", linewidth=2)
    ax.scatter(waypoints[:, 0], waypoints[:, 1], waypoints[:, 2],
               color='red', label='Waypoints', s=50)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Minimum Snap Trajectory with Soft Waypoint Constraints")
    ax.legend()
    plt.tight_layout()
    plt.show()
else:
    print("❌ Optimization failed. Status:", problem.status)
