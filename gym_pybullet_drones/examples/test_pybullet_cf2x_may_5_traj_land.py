import numpy as np
import pybullet as p
import pybullet_data
import time # as simulation time needs a delay
import os
import argparse
from datetime import datetime
import pdb
import math
import random
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from gym_pybullet_drones.utils.enums import DroneModel, Physics
import pandas as pd



p.connect(p.GUI)    # if we want no visuals p.connect(p.DIRECT)
p.resetSimulation() 
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)  # (x, y, z)
p.setRealTimeSimulation(0)



## Load Assets # Check doccumentation to load file
p.loadURDF("plane.urdf", [0 , 0 ,0 ] , [0 , 0 , 0 , 1 ])    # Ground Plane (Not to have everything fall) (position_space(x,y,z), orientation(quaternion))
targid = p.loadURDF("/home/ziad/gym-pybullet-drones/gym_pybullet_drones/assets/cf2x.urdf", [0 , 0 , 0 ] , [0 , 0 , 0 , 1 ] , useFixedBase = False)   # (file_name, position_space(x,y,z), orientation(quaternion) ) (FixBase to make the base fixed)
obstacle_right = p.loadURDF("/home/ziad/gym-pybullet-drones/gym_pybullet_drones/assets/block.urdf", [1 , 1.5 , 0 ] , [0 , 0 , 0 , 1 ] , useFixedBase = True)
obstacle_left = p.loadURDF("/home/ziad/gym-pybullet-drones/gym_pybullet_drones/assets/block.urdf", [1 , 0.8 , 0 ] , [0 , 0 , 0 , 1 ] , useFixedBase = True)
landing_mark = p.loadURDF("/home/ziad/gym-pybullet-drones/gym_pybullet_drones/assets/landing_mark.urdf", [3 , 1.25 , 0 ] , [0 , 0 , 0 , 1 ] , useFixedBase = True)
landing_mark_yellow = p.loadURDF("/home/ziad/gym-pybullet-drones/gym_pybullet_drones/assets/landing_mark_yellow.urdf", [0 , 0 , 0 ] , [0 , 0 , 0 , 1 ] , useFixedBase = True)
obj_of_focus = targid   # To Focus on this specific asset
#time.sleep(10)

print(p.getNumJoints(targid)) #to get number of joints of the urdf = 12
for i in range(p.getNumJoints(targid)): # If we wanted the info of each joint
    print(p.getJointInfo(targid , i))

'''
(0, b'prop0_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'prop0_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), -1)     # Planar Joint
(1, b'prop1_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'prop1_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), -1)
(2, b'prop2_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'prop2_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), -1)
(3, b'prop3_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'prop3_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), -1)
(4, b'center_of_mass_joint', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'center_of_mass_link', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), -1)

'''    

jointid = 0 # The joint number we want to get its value 
prop_lower_pos = p.getJointInfo(targid, jointid ) [8]   # Minimum angle that the joint can go (in index 8)
prop_upper_pos = p.getJointInfo(targid, jointid ) [9]   # Maximum angle that the joint can go (in index 9)
# Maximum force specified in URDF (possibly other file formats) Note that this value is not automatically used. You can use maxForce in 'setJointMotorControl2'.
prop_max_force = p.getJointInfo(targid, jointid ) [10]   # Maximum Force (in index 10)
# Maximum velocity specified in URDF. Note that the maximum velocity is not used in actual motor control commands at the moment.
prop_max_vel = p.getJointInfo(targid, jointid ) [11]   # Maximum Velocity (in index 10)

print("prop_lower_pos: ", prop_lower_pos)
print("prop_upper_pos: ", prop_upper_pos)
print("prop_max_force: ", prop_max_force)
print("prop_max_vel: ", prop_max_vel)





### Velocity Hybrid Control - Just Worked as good as the above but better with the z-hight start - best method till now###

# === Load trajectory reference ===
position_refs = pd.read_csv("bezier_position.csv", header=0).astype(float).values.tolist()
velocity_refs = pd.read_csv("bezier_velocity.csv", header=0).astype(float).values.tolist()
assert len(position_refs) == len(velocity_refs)


# === Gains ===
# Older kp_xy = 10.0
Kp_xy = 28.0   # Increase to pull drone harder toward the path
Kd_xy = 5.0    # Increase to dampen overshoot and reduce lag

## Older - Best Values for the thrust
# Kp_thrust = 35.0   # Stronger response to height errors
# Kd_thrust = 75.5
# Ki_thrust = 58.0  # Helps reduce long-term offset

Kp_thrust = 105.0   # Stronger response to height errors
Kd_thrust = 1155.5
Ki_thrust = 610.0  # Helps reduce long-term offset

Kp_attitude = 2.0
Kd_attitude = 1.0
Kp_yaw = 1.0

# === Altitude / Thrust Settings ===
thrust = 9.8
max_thrust = 28.9   # Play with it till 20 to get better results - was 18.9 with older best result
min_thrust = -20.1
thrust_change_limit = 20.0 
max_altitude = 1.5
min_altitude = 0.1

# === Control state ===
prev_error_z = 0
integral_error_z = 0
prev_error_roll = 0
prev_error_pitch = 0
prev_error_yaw = 0

# === Logging ===
altitude_log = []
thrust_log = []
x_log = []
y_log = []
z_log = []

landing_mode = False
landing_counter = 0
max_landing_steps = 100  # number of steps over which to reduce thrust


takeoff_z = position_refs[0][2]
takeoff_reached = False
print("Final landing Z:", position_refs[-1][2])
print("Final position refs:", position_refs[-10:])
print("Final velocity refs:", velocity_refs[-10:])

# === Takeoff phase ===
while True:
    pos, _ = p.getBasePositionAndOrientation(targid)
    if pos[2] >= takeoff_z - 0.01:
        print("✅ Takeoff complete — starting trajectory tracking.")
        break  # Exit the takeoff loop
    
    thrust = 11.0  # Strong constant lift
    p.resetBaseVelocity(targid, [0, 0, 0], [0, 0, 0])
    p.applyExternalForce(targid, -1, [0, 0, thrust], [0, 0, 0], p.LINK_FRAME)
    p.stepSimulation()
    time.sleep(0.01)


# === Main Loop ===
for step in range(len(position_refs)):
    p.resetBaseVelocity(targid, [0, 0, 0], [0, 0, 0])
    pos, orn = p.getBasePositionAndOrientation(targid)
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)
    lin_vel, _ = p.getBaseVelocity(targid)
    vx, vy, vz = lin_vel

    # === Reference State ===
    ref_pos = np.array(position_refs[step])
    ref_vel = np.array(velocity_refs[step])
    pos_arr = np.array(pos)


    # === Altitude Control ===
    error_z = ref_pos[2] - pos[2]
    d_error_z = error_z - prev_error_z
    integral_error_z += error_z
    integral_error_z = np.clip(integral_error_z, -0.5, 0.5)
    target_thrust = 9.8 + (Kp_thrust * error_z) + (Kd_thrust * d_error_z) + (Ki_thrust * integral_error_z)

    thrust += np.clip(target_thrust - thrust, -thrust_change_limit, thrust_change_limit)
    thrust = np.clip(thrust, min_thrust, max_thrust)
    prev_error_z = error_z

    # # === Landing Phase Trigger ===
    # if step > len(position_refs) - 30:  # Last few steps
    #     landing_mode = True

    # if landing_mode:
    #     error_z = 0.0 - pos[2]  # Target is ground (z = 0)
    #     d_error_z = error_z - prev_error_z
    #     integral_error_z += error_z
    #     integral_error_z = np.clip(integral_error_z, -0.5, 0.5)

    #     target_thrust = 9.8 + (Kp_thrust * error_z) + (Kd_thrust * d_error_z) + (Ki_thrust * integral_error_z)
    #     thrust += np.clip(target_thrust - thrust, -thrust_change_limit, thrust_change_limit)
    #     #thrust = np.clip(thrust, 0.0, max_thrust)  #  Prevent negative thrust

    #     x_force = 0.0  # Reduce horizontal forces
    #     y_force = 0.0

    #     if pos[2] <= 0.12 and abs(vz) < 0.05:
    #         print("✅ Landing complete")
    #         break

    # === Hybrid XY Control (Position + Velocity) ===
    error_pos = ref_pos[:2] - pos_arr[:2]
    error_vel = ref_vel[:2] - np.array([vx, vy])
    x_force = Kp_xy * error_pos[0] + Kd_xy * error_vel[0]
    y_force = Kp_xy * error_pos[1] + Kd_xy * error_vel[1]

    # === Yaw Control ===
    error_yaw = -yaw
    d_error_yaw = error_yaw - prev_error_yaw
    yaw_torque = Kp_yaw * error_yaw + 0.5 * d_error_yaw
    prev_error_yaw = error_yaw

    # === Attitude Control ===
    error_roll = -roll
    error_pitch = -pitch
    d_error_roll = error_roll - prev_error_roll
    d_error_pitch = error_pitch - prev_error_pitch
    roll_torque = Kp_attitude * error_roll + Kd_attitude * d_error_roll
    pitch_torque = Kp_attitude * error_pitch + Kd_attitude * d_error_pitch
    prev_error_roll = error_roll
    prev_error_pitch = error_pitch

    # === Apply === # 
    p.applyExternalForce(targid, -1, [x_force, y_force, thrust], [0, 0, 0], p.LINK_FRAME)
    p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, yaw_torque], p.LINK_FRAME)

    # === Log ===
    altitude_log.append(pos[2])
    thrust_log.append(thrust)
    x_log.append(pos[0])
    y_log.append(pos[1])
    z_log.append(pos[2])

    print(f"Step {step} | Z: {pos[2]:.2f} | Thrust: {thrust:.2f} | Error XY: {error_pos}")

    p.stepSimulation()
    time.sleep(0.01)

# # Cut off motor/force after landing
# p.resetBaseVelocity(targid, [0, 0, 0], [0, 0, 0])
# for _ in range(110):
#     p.applyExternalForce(targid, -1, [0, 0, 0], [0, 0, 0], p.LINK_FRAME)
#     p.stepSimulation()
#     time.sleep(0.01)




# === XY, Altitude, and Thrust Plots ===
plt.figure(figsize=(12, 8))

# Altitude plot
plt.subplot(3, 1, 1)
plt.plot(altitude_log, label="Altitude (Z)")
plt.axhline(y=max_altitude, color='r', linestyle='--', label="Max Altitude")
plt.axhline(y=min_altitude, color='gray', linestyle='--', label="Min Altitude")
plt.ylabel("Z (m)")
plt.title("Drone Altitude Over Time")
plt.legend()

# Thrust plot
plt.subplot(3, 1, 2)
plt.plot(thrust_log, label="Thrust", color='green')
plt.ylabel("Thrust (N)")
plt.title("Thrust Command Over Time")
plt.legend()

# 2D XY path comparison
plt.subplot(3, 1, 3)
plt.plot(x_log, y_log, label="Actual XY Path", linewidth=2)
way_x = [wp[0] for wp in position_refs]
way_y = [wp[1] for wp in position_refs]
plt.plot(way_x, way_y, '--', color='orange', label="Reference XY Path")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("XY Path Comparison")
plt.legend()
plt.tight_layout()
plt.show()

# === 3D Trajectory Comparison ===
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')
ax.plot(x_log, y_log, z_log, label="Actual Trajectory", color='blue')
ref_x = [pt[0] for pt in position_refs]
ref_y = [pt[1] for pt in position_refs]
ref_z = [pt[2] for pt in position_refs]
ax.plot(ref_x, ref_y, ref_z, '--', color='orange', label="Reference Trajectory")

ax.set_title("3D Trajectory Comparison")
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.legend()
plt.show()









################################################ A copy of last code to test the landing phase #################################################### 

# ### Velocity Hybrid Control - Just Worked as good as the above but better with the z-hight start - best method till now###

# # === Load trajectory reference ===
# position_refs = pd.read_csv("bezier_position.csv", header=0).astype(float).values.tolist()
# velocity_refs = pd.read_csv("bezier_velocity.csv", header=0).astype(float).values.tolist()
# assert len(position_refs) == len(velocity_refs)


# # === Gains ===
# Kp_xy = 10.0   # Increase to pull drone harder toward the path
# Kd_xy = 5.0    # Increase to dampen overshoot and reduce lag
# Kp_thrust = 10.0   # Stronger response to height errors
# Kd_thrust = 2.5
# Ki_thrust = 18.0  # Helps reduce long-term offset
# Kp_attitude = 2.0
# Kd_attitude = 1.0
# Kp_yaw = 1.0

# # === Altitude / Thrust Settings ===
# thrust = 9.8
# max_thrust = 18.9   # Play with it till 20 to get better results
# min_thrust = -20.0
# thrust_change_limit = 20.0 
# max_altitude = 1.5
# min_altitude = 0.1

# # === Control state ===
# prev_error_z = 0
# integral_error_z = 0
# prev_error_roll = 0
# prev_error_pitch = 0
# prev_error_yaw = 0

# # === Logging ===
# altitude_log = []
# thrust_log = []
# x_log = []
# y_log = []
# z_log = []

# landing_mode = False
# landing_counter = 0
# max_landing_steps = 100  # number of steps over which to reduce thrust


# takeoff_z = position_refs[0][2]
# takeoff_reached = False
# print("Final landing Z:", position_refs[-1][2])
# print("Final position refs:", position_refs[-10:])
# print("Final velocity refs:", velocity_refs[-10:])

# # === Takeoff phase ===
# while True:
#     pos, _ = p.getBasePositionAndOrientation(targid)
#     if pos[2] >= takeoff_z - 0.05:
#         print("✅ Takeoff complete — starting trajectory tracking.")
#         break  # Exit the takeoff loop

#     thrust = 11.0  # Strong constant lift
#     p.resetBaseVelocity(targid, [0, 0, 0], [0, 0, 0])
#     p.applyExternalForce(targid, -1, [0, 0, thrust], [0, 0, 0], p.LINK_FRAME)
#     p.stepSimulation()
#     time.sleep(0.01)

# # === Main Loop ===
# for step in range(len(position_refs)):
#     p.resetBaseVelocity(targid, [0, 0, 0], [0, 0, 0])
    
    
#     pos, orn = p.getBasePositionAndOrientation(targid)
#     roll, pitch, yaw = p.getEulerFromQuaternion(orn)
#     lin_vel, _ = p.getBaseVelocity(targid)
#     vx, vy, vz = lin_vel

#     # === Reference State ===
#     ref_pos = np.array(position_refs[step])
#     ref_vel = np.array(velocity_refs[step])
#     pos_arr = np.array(pos)


#     # === Altitude Control ===
#     error_z = ref_pos[2] - pos[2]
#     d_error_z = error_z - prev_error_z
#     integral_error_z += error_z
#     integral_error_z = np.clip(integral_error_z, -0.5, 0.5)
#     target_thrust = 9.8 + (Kp_thrust * error_z) + (Kd_thrust * d_error_z) + (Ki_thrust * integral_error_z)

#     thrust += np.clip(target_thrust - thrust, -thrust_change_limit, thrust_change_limit)
#     thrust = np.clip(thrust, min_thrust, max_thrust)
#     prev_error_z = error_z


#     # === Hybrid XY Control (Position + Velocity) ===
#     error_pos = ref_pos[:2] - pos_arr[:2]
#     error_vel = ref_vel[:2] - np.array([vx, vy])
#     x_force = Kp_xy * error_pos[0] + Kd_xy * error_vel[0]
#     y_force = Kp_xy * error_pos[1] + Kd_xy * error_vel[1]

#     # === Yaw Control ===
#     error_yaw = -yaw
#     d_error_yaw = error_yaw - prev_error_yaw
#     yaw_torque = Kp_yaw * error_yaw + 0.5 * d_error_yaw
#     prev_error_yaw = error_yaw

#     # === Attitude Control ===
#     error_roll = -roll
#     error_pitch = -pitch
#     d_error_roll = error_roll - prev_error_roll
#     d_error_pitch = error_pitch - prev_error_pitch
#     roll_torque = Kp_attitude * error_roll + Kd_attitude * d_error_roll
#     pitch_torque = Kp_attitude * error_pitch + Kd_attitude * d_error_pitch
#     prev_error_roll = error_roll
#     prev_error_pitch = error_pitch

#     # === Apply === #
#     p.applyExternalForce(targid, -1, [x_force, y_force, thrust], [0, 0, 0], p.LINK_FRAME)
#     p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, yaw_torque], p.LINK_FRAME)

#     # === Log ===
#     altitude_log.append(pos[2])
#     thrust_log.append(thrust)
#     x_log.append(pos[0])
#     y_log.append(pos[1])
#     z_log.append(pos[2])

#     print(f"Step {step} | Z: {pos[2]:.2f} | Thrust: {thrust:.2f} | Error XY: {error_pos}")

#     p.stepSimulation()
#     time.sleep(0.01)



# # === XY, Altitude, and Thrust Plots ===
# plt.figure(figsize=(12, 8))

# # Altitude plot
# plt.subplot(3, 1, 1)
# plt.plot(altitude_log, label="Altitude (Z)")
# plt.axhline(y=max_altitude, color='r', linestyle='--', label="Max Altitude")
# plt.axhline(y=min_altitude, color='gray', linestyle='--', label="Min Altitude")
# plt.ylabel("Z (m)")
# plt.title("Drone Altitude Over Time")
# plt.legend()

# # Thrust plot
# plt.subplot(3, 1, 2)
# plt.plot(thrust_log, label="Thrust", color='green')
# plt.ylabel("Thrust (N)")
# plt.title("Thrust Command Over Time")
# plt.legend()

# # 2D XY path comparison
# plt.subplot(3, 1, 3)
# plt.plot(x_log, y_log, label="Actual XY Path", linewidth=2)
# way_x = [wp[0] for wp in position_refs]
# way_y = [wp[1] for wp in position_refs]
# plt.plot(way_x, way_y, '--', color='orange', label="Reference XY Path")
# plt.xlabel("X (m)")
# plt.ylabel("Y (m)")
# plt.title("XY Path Comparison")
# plt.legend()
# plt.tight_layout()
# plt.show()

# # === 3D Trajectory Comparison ===
# fig = plt.figure(figsize=(10, 7))
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(x_log, y_log, z_log, label="Actual Trajectory", color='blue')
# ref_x = [pt[0] for pt in position_refs]
# ref_y = [pt[1] for pt in position_refs]
# ref_z = [pt[2] for pt in position_refs]
# ax.plot(ref_x, ref_y, ref_z, '--', color='orange', label="Reference Trajectory")

# ax.set_title("3D Trajectory Comparison")
# ax.set_xlabel("X (m)")
# ax.set_ylabel("Y (m)")
# ax.set_zlabel("Z (m)")
# ax.legend()
# plt.show()