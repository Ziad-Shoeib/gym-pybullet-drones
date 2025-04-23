# Following This Tutorial: https://www.youtube.com/watch?v=kZxPaGdoSJY

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


p.connect(p.GUI)    # if we want no visuals p.connect(p.DIRECT)
p.resetSimulation() 
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)  # (x, y, z)
p.setRealTimeSimulation(0)



## Load Assets # Check doccumentation to load file
p.loadURDF("plane.urdf", [0 , 0 ,0 ] , [0 , 0 , 0 , 1 ])    # Ground Plane (Not to have everything fall) (position_space(x,y,z), orientation(quaternion))
targid = p.loadURDF("/home/ziad/gym-pybullet-drones/gym_pybullet_drones/assets/cf2x.urdf", [0 , 0 , 0 ] , [0 , 0 , 0 , 1 ] , useFixedBase = False)   # (file_name, position_space(x,y,z), orientation(quaternion) ) (FixBase to make the base fixed)
obstacle_right = p.loadURDF("/home/ziad/gym-pybullet-drones/gym_pybullet_drones/assets/block.urdf", [1 , 1.5 , 0 ] , [0 , 0 , 0 , 1 ] , useFixedBase = True)
obstacle_left = p.loadURDF("/home/ziad/gym-pybullet-drones/gym_pybullet_drones/assets/block.urdf", [1 , 1 , 0 ] , [0 , 0 , 0 , 1 ] , useFixedBase = True)
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

'''
prop_lower_pos:  0.0
prop_upper_pos:  -1.0
prop_max_force:  0.0
prop_max_vel:  0.0

'''
'''
If you check the output of getJointInfo(), you'll see:

    Joint type = 4 (which corresponds to p.JOINT_FIXED).
    Max force = 0 and Max velocity = 0, meaning these joints aren't set up for active control in the URDF.

    In gym-pybullet-drones, thrust is applied using:
p.applyExternalForce(bodyUniqueId=targid, 
                     linkIndex=-1,  # -1 means applied to the base (drone body)
                     forceObj=[0, 0, thrust],  # Thrust force in Newtons
                     posObj=[0, 0, 0],  # Apply at the center of mass
                     flags=p.WORLD_FRAME)

'''

## gym-pubullet-drones doesn't use this method
# for step in range(5000):
#     # joint_two_targ = np.random.uniform(jlower , jupper) # Just Sampling new target positions
#     # joint_four_targ = np.random.uniform(jlower , jupper)
#     #p.setJointMotorControlArray(targid, [0 , 1 , 2 , 3 ] , p.POSITION_CONTROL, targetPositions = [ 0.0 , 0.0 , 0.0 , 0.0 ])  # Still does not change positions
    
#     p.setJointMotorControlArray(targid, [0 , 1 , 2 , 3 ] , p.VELOCITY_CONTROL, [10.0 , 10.0 , 10.0 , 10.0] )  # Failed
#     #p.setJointMotorControlArray(targid, [0] , p.VELOCITY_CONTROL, [100] )  # Apply forces to change the position
    
#     #print(p.getJointStates(targid , [ 2 , 4 ]))
#     #print(p.getLinkStates(targid , [ 2 , 4 ]))
#     focus_position , focus_orientation = p.getBasePositionAndOrientation(targid)    # Determine the position and orientation of our target_id   
#     #p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-10, cameraTargetPosition = focus_position) # To Focus the camera on the focus position
#     p.stepSimulation()
#     time.sleep(.01)


'''
Is gym-pybullet-drones Using Forces in the World Frame or Body Frame?

The repository primarily applies forces and torques in the body frame (LINK_FRAME) rather than the world frame (WORLD_FRAME).

1. Force Application in BaseAviary.py
Inside BaseAviary.py, the method applyExternalForce() is used:
pyb.applyExternalForce(self.DRONE_IDS[i], 
                       linkIndex=-1, 
                       forceObj=[0, 0, thrust], 
                       posObj=[0, 0, 0], 
                       flags=pyb.LINK_FRAME)
Torque Application in BaseAviary.py
Similarly, torques are applied like this:
pyb.applyExternalTorque(self.DRONE_IDS[i], 
                        linkIndex=-1, 
                        torqueObj=[roll_torque, pitch_torque, yaw_torque], 
                        flags=pyb.LINK_FRAME)

'''







# ### Trial 2: Much better but the drone does not land exactly and overshoots a bit
# # **PID gains (Tuned for altitude stability)**
# Kp_thrust = 0.3  # **Smoother altitude correction**
# Kd_thrust = 1.2  # **Better damping**
# Ki_thrust = 0.00002  # **Very small integral gain**

# Kp_attitude = 2.0  
# Kd_attitude = 1.0  

# Kp_xy = 0.3  
# Kd_xy = 0.1  

# Kp_yaw = 1.0  # **New yaw control gain**

# # **Waypoints (Max altitude = 1.5m)**
# waypoints = [[1.5, 1.25, 1.3],  
#              [3.0, 1.25, 1.3],  
#              [3.0, 1.25, 0.1]]  

# current_waypoint = 0

# # **Set Maximum Altitude**
# max_altitude = 1.5  
# min_altitude = 0.1  

# # **Thrust Limits**
# thrust = 9.8  
# max_thrust = 9.9  
# min_thrust = 5.0  
# thrust_change_limit = 0.02  

# # **Initialize variables**
# prev_error_z = 0
# integral_error_z = 0  
# prev_error_x = 0
# prev_error_y = 0
# prev_error_roll = 0
# prev_error_pitch = 0
# prev_error_yaw = 0  # **Added yaw control**

# # Logging
# altitude_log = []
# thrust_log = []
# x_log = []
# y_log = []

# for step in range(5000):
#     # **Reset accumulated forces to remove unwanted motion**
#     p.resetBaseVelocity(targid, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])

#     # Get drone position & orientation
#     pos, orn = p.getBasePositionAndOrientation(targid)
#     roll, pitch, yaw = p.getEulerFromQuaternion(orn)

#     print(f"Drone Position: {pos}")  

#     #### ✅ **Check if waypoint reached**
#     target_x, target_y, target_z = waypoints[current_waypoint]

#     # **Override thrust dynamically instead of setting it to a fixed value**
#     if pos[2] > max_altitude:
#         target_z = max_altitude  
#         thrust -= 0.01  # **Reduce thrust step by step**
    
#     # **Prevent crashing into the ground**
#     if pos[2] < min_altitude:
#         target_z = min_altitude
#         thrust += 0.01  # **Increase thrust step by step**

#     dist = np.linalg.norm([target_x - pos[0], target_y - pos[1], target_z - pos[2]])

#     if dist < 0.2 and current_waypoint < len(waypoints) - 1:
#         current_waypoint += 1  

#     #### ✅ **Altitude Control (Thrust)**
#     error_z = target_z - pos[2]
#     d_error_z = error_z - prev_error_z
#     integral_error_z = max(-0.05, min(0.05, error_z))  

#     # **Use normal thrust control if altitude is within bounds**
#     if pos[2] <= max_altitude:
#         target_thrust = 9.8 + (Kp_thrust * error_z) + (Kd_thrust * d_error_z) + (Ki_thrust * integral_error_z)
#         thrust += np.clip(target_thrust - thrust, -thrust_change_limit, thrust_change_limit)
#         thrust = max(min_thrust, min(thrust, max_thrust))  

#     prev_error_z = error_z  

#     print(f"Thrust: {thrust}")  

#     #### ✅ **Position Control (X-Y Movement)**
#     error_x = target_x - pos[0]
#     error_y = target_y - pos[1]

#     d_error_x = error_x - prev_error_x
#     d_error_y = error_y - prev_error_y

#     # **Movement Force in WORLD_FRAME**
#     x_force = 3 * error_x  
#     y_force = 3 * error_y  

#     prev_error_x = error_x  
#     prev_error_y = error_y  

#     #### ✅ **Yaw Control (New)**
#     error_yaw = -yaw  # **We want yaw = 0**
#     d_error_yaw = error_yaw - prev_error_yaw
#     yaw_torque = Kp_yaw * error_yaw + 0.5 * d_error_yaw  # **Yaw control to keep drone aligned**
#     prev_error_yaw = error_yaw

#     #### ✅ **Attitude Control (Roll & Pitch Stabilization)**
#     error_roll = -roll  
#     error_pitch = -pitch  

#     d_error_roll = error_roll - prev_error_roll
#     d_error_pitch = error_pitch - prev_error_pitch

#     roll_torque = Kp_attitude * error_roll + Kd_attitude * d_error_roll
#     pitch_torque = Kp_attitude * error_pitch + Kd_attitude * d_error_pitch

#     prev_error_roll = error_roll
#     prev_error_pitch = error_pitch

#     #### ✅ **Apply Forces & Torques**
#     p.applyExternalForce(targid, -1, [x_force, y_force, thrust], [0, 0, 0], p.LINK_FRAME)  
#     p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, yaw_torque], p.LINK_FRAME)

#     #### ✅ **Logging**
#     pos, orn = p.getBasePositionAndOrientation(targid)
#     print("Drone Position:", pos)
#     print("Drone Orientation:", orn)    
#     altitude_log.append(pos[2])
#     thrust_log.append(thrust)
#     x_log.append(pos[0])
#     y_log.append(pos[1])

#     p.stepSimulation()
#     time.sleep(0.01)

# # **Results**
# plt.figure(figsize=(12, 6))
# plt.subplot(3, 1, 1)
# plt.plot(altitude_log, label="Altitude (m)")
# plt.axhline(y=max_altitude, color="r", linestyle="--", label="Max Altitude")
# plt.legend()
# plt.ylabel("Altitude (m)")
# plt.title("Drone Altitude Over Time")

# plt.subplot(3, 1, 2)
# plt.plot(thrust_log, label="Thrust (N)", color="g")
# plt.legend()
# plt.ylabel("Thrust (N)")
# plt.title("Thrust Over Time")

# plt.subplot(3, 1, 3)
# plt.plot(x_log, y_log, label="Drone Path")
# plt.scatter([1.5, 3.0], [1.25, 1.25], color="r", marker="x", label="Waypoints")
# plt.scatter(3.0, 1.25, color="g", marker="o", label="Landing Mark")
# plt.legend()
# plt.xlabel("X Position (m)")
# plt.ylabel("Y Position (m)")
# plt.title("Drone X-Y Movement")

# plt.show()



### Trial 3: Adding More waypoints -- The Most Succeful Rill now
# **PID gains (Tuned for altitude stability)**
Kp_thrust = 0.4
Kd_thrust = 2.0
Ki_thrust = 0.002

Kp_attitude = 2.0  
Kd_attitude = 1.0  

Kp_xy = 0.3  
Kd_xy = 0.1  

Kp_yaw = 1.0  # **New yaw control gain**

#**Waypoints (Max altitude = 1.5m)**
waypoints = [
    [0.0, 0.0, 0.5],    # Takeoff straight up
    [0.5, 0.5, 1.0],    # Start moving forward & up
    [1.0, 1.25, 1.5],   # Approach gap entrance at center
    [1.5, 1.25, 1.5],   # Midway inside the gap
    [2.0, 1.25, 1.5],   # Exit the gap
    [2.5, 1.25, 1.3],   # Move forward
    [3.0, 1.25, 1.0],   # Approach landing
    [3.0, 1.25, 0.5],   # Descend partway
    [3.0, 1.25, 0.1],   # Final land
]


current_waypoint = 0

# **Set Maximum Altitude**
max_altitude = 1.0  
min_altitude = 0.1  

# **Thrust Limits**
thrust = 9.8  
max_thrust = 9.9  
min_thrust = 0.01  
thrust_change_limit = 0.8  

# **Initialize variables**
prev_error_z = 0
integral_error_z = 0  
prev_error_x = 0
prev_error_y = 0
prev_error_roll = 0
prev_error_pitch = 0
prev_error_yaw = 0  # **Added yaw control**

# Logging
altitude_log = []
thrust_log = []
x_log = []
y_log = []

for step in range(15000):
    # **Reset accumulated forces to remove unwanted motion**
    p.resetBaseVelocity(targid, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])

    # Get drone position & orientation
    pos, orn = p.getBasePositionAndOrientation(targid)
    roll, pitch, yaw = p.getEulerFromQuaternion(orn)

    print(f"Drone Position: {pos}")  

    #### ✅ **Check if waypoint reached**
    target_x, target_y, target_z = waypoints[current_waypoint]

    # **Override thrust dynamically instead of setting it to a fixed value**
    if pos[2] > max_altitude:
        target_z = max_altitude  
        thrust -= 0.01  # **Reduce thrust step by step**
    
    # **Prevent crashing into the ground**
    if pos[2] < min_altitude:
        target_z = min_altitude
        thrust += 0.01  # **Increase thrust step by step**

    dist = np.linalg.norm([target_x - pos[0], target_y - pos[1], target_z - pos[2]])

    if dist < 0.1 and current_waypoint < len(waypoints) - 1:
        current_waypoint += 1  

    #### ✅ **Altitude Control (Thrust)**
    error_z = target_z - pos[2]
    d_error_z = error_z - prev_error_z
    #integral_error_z = max(-0.05, min(0.05, error_z))  
    integral_error_z += error_z  # accumulate
    integral_error_z = np.clip(integral_error_z, -0.5, 0.5)

    # **Use normal thrust control if altitude is within bounds**
    if pos[2] <= max_altitude:
        target_thrust = 9.8 + (Kp_thrust * error_z) + (Kd_thrust * d_error_z) + (Ki_thrust * integral_error_z)
        thrust += np.clip(target_thrust - thrust, -thrust_change_limit, thrust_change_limit)
        thrust = max(min_thrust, min(thrust, max_thrust))  

    prev_error_z = error_z  

    print(f"Thrust: {thrust}")  

    #### ✅ **Position Control (X-Y Movement)**
    error_x = target_x - pos[0]
    error_y = target_y - pos[1]

    d_error_x = error_x - prev_error_x
    d_error_y = error_y - prev_error_y

    # **Movement Force in WORLD_FRAME**
    x_force = 2.0 * error_x  
    y_force = 2.0 * error_y  

    prev_error_x = error_x  
    prev_error_y = error_y  

    #### ✅ **Yaw Control (New)**
    error_yaw = -yaw  # **We want yaw = 0**
    d_error_yaw = error_yaw - prev_error_yaw
    yaw_torque = Kp_yaw * error_yaw + 0.5 * d_error_yaw  # **Yaw control to keep drone aligned**
    prev_error_yaw = error_yaw

    #### ✅ **Attitude Control (Roll & Pitch Stabilization)**
    error_roll = -roll  
    error_pitch = -pitch  

    d_error_roll = error_roll - prev_error_roll
    d_error_pitch = error_pitch - prev_error_pitch

    roll_torque = Kp_attitude * error_roll + Kd_attitude * d_error_roll
    pitch_torque = Kp_attitude * error_pitch + Kd_attitude * d_error_pitch

    prev_error_roll = error_roll
    prev_error_pitch = error_pitch

    #### ✅ **Apply Forces & Torques**
    p.applyExternalForce(targid, -1, [x_force, y_force, thrust], [0, 0, 0], p.LINK_FRAME)  
    p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, yaw_torque], p.LINK_FRAME)

    #### ✅ **Logging**
    pos, orn = p.getBasePositionAndOrientation(targid)
    print("Drone Position:", pos)
    print("Drone Orientation:", orn)    
    altitude_log.append(pos[2])
    thrust_log.append(thrust)
    x_log.append(pos[0])
    y_log.append(pos[1])

    p.stepSimulation()
    time.sleep(0.01)






# Trial 4, Refining Trial 3 - Failed
# # === Constants and Gains ===
# Kp_attitude = 2.0
# Kd_attitude = 1.0

# Kp_xy = 0.3
# Kd_xy = 0.1

# Kp_yaw = 1.0

# # === Waypoints ===


# # === Original coarse waypoints ===
# coarse_waypoints = [
#     [0.0, 0.0, 0.5],
#     [0.5, 0.5, 1.0],
#     [1.0, 1.25, 1.5],
#     [1.5, 1.25, 1.5],
#     [2.0, 1.25, 1.5],
#     [2.5, 1.25, 1.3],
#     [3.0, 1.25, 1.0],
#     [3.0, 1.25, 0.5],
#     [3.0, 1.25, 0.1],
# ]

# # === Interpolation function ===
# def interpolate_waypoints(wp_list, points_between=5):
#     result = []
#     for i in range(len(wp_list) - 1):
#         start = np.array(wp_list[i])
#         end = np.array(wp_list[i + 1])
#         for alpha in np.linspace(0, 1, points_between, endpoint=False):
#             point = (1 - alpha) * start + alpha * end
#             result.append(point.tolist())
#     result.append(wp_list[-1])  # Add final point
#     return result

# # === Generate fine-grained path ===
# waypoints = interpolate_waypoints(coarse_waypoints, points_between=10)  # ~90 waypoints
# current_waypoint = 0

# # === Thrust settings ===
# thrust = 9.8
# thrust_lift = 11.0
# thrust_hover = 9.8
# thrust_descend = 0.4

# # === Control variables ===
# prev_error_x = 0
# prev_error_y = 0
# prev_error_roll = 0
# prev_error_pitch = 0
# prev_error_yaw = 0

# # === Logs ===
# altitude_log = []
# thrust_log = []
# x_log = []
# y_log = []

# for step in range(15000):
#     p.resetBaseVelocity(targid, [0, 0, 0], [0, 0, 0])
#     pos, orn = p.getBasePositionAndOrientation(targid)
#     roll, pitch, yaw = p.getEulerFromQuaternion(orn)

#     # === Waypoint Switching ===
#     target_x, target_y, target_z = waypoints[current_waypoint]
#     dist = np.linalg.norm([target_x - pos[0], target_y - pos[1], target_z - pos[2]])

#     if dist < 0.1 and current_waypoint < len(waypoints) - 1:
#         current_waypoint += 1

#     # === Thrust Scheduler ===
#     z = pos[2]
#     if z < target_z - 0.2:
#         thrust = thrust_lift
#     elif abs(z - target_z) < 0.1:
#         thrust = thrust_hover
#     elif z > target_z + 0.2:
#         thrust = thrust_descend

#     thrust = np.clip(thrust, 0.4, 11.0)

#     print(f"Step: {step} | Z: {z:.2f} | Target Z: {target_z:.2f} | Thrust: {thrust:.2f}")

#     # === XY Position Control ===
#     error_x = target_x - pos[0]
#     error_y = target_y - pos[1]

#     d_error_x = error_x - prev_error_x
#     d_error_y = error_y - prev_error_y

#     x_force = 2.0 * error_x
#     y_force = 2.0 * error_y

#     prev_error_x = error_x
#     prev_error_y = error_y

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

#     # === Apply Forces & Torques ===
#     p.applyExternalForce(targid, -1, [x_force, y_force, thrust], [0, 0, 0], p.LINK_FRAME)
#     p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, yaw_torque], p.LINK_FRAME)

#     # === Logging ===
#     altitude_log.append(z)
#     thrust_log.append(thrust)
#     x_log.append(pos[0])
#     y_log.append(pos[1])

#     p.stepSimulation()
#     time.sleep(0.01)

# ## Trial 4.2

# # === Constants and Gains ===
# Kp_attitude = 2.0
# Kd_attitude = 1.0

# Kp_xy = 0.3
# Kd_xy = 0.1

# Kp_yaw = 1.0

# # === Coarse Waypoints ===
# coarse_waypoints = [
#     [0.0, 0.0, 0.5],
#     [0.5, 0.5, 1.0],
#     [1.0, 1.25, 1.5],
#     [1.5, 1.25, 1.5],
#     [2.0, 1.25, 1.5],
#     [2.5, 1.25, 1.3],
#     [3.0, 1.25, 1.0],
#     [3.0, 1.25, 0.5],
#     [3.0, 1.25, 0.1],
# ]

# # === Interpolation ===
# def interpolate_waypoints(wp_list, points_between=10):
#     result = []
#     for i in range(len(wp_list) - 1):
#         start = np.array(wp_list[i])
#         end = np.array(wp_list[i + 1])
#         for alpha in np.linspace(0, 1, points_between, endpoint=False):
#             point = (1 - alpha) * start + alpha * end
#             result.append(point.tolist())
#     result.append(wp_list[-1])
#     return result

# waypoints = interpolate_waypoints(coarse_waypoints, points_between=10)
# current_waypoint = 0

# # === Control State ===
# prev_error_x = 0
# prev_error_y = 0
# prev_error_roll = 0
# prev_error_pitch = 0
# prev_error_yaw = 0

# # === Logging ===
# altitude_log = []
# thrust_log = []
# x_log = []
# y_log = []

# for step in range(1000):
#     p.resetBaseVelocity(targid, [0, 0, 0], [0, 0, 0])
#     pos, orn = p.getBasePositionAndOrientation(targid)
#     roll, pitch, yaw = p.getEulerFromQuaternion(orn)

#     # === Waypoint Target ===
#     target_x, target_y, target_z = waypoints[current_waypoint]
#     dist = np.linalg.norm([target_x - pos[0], target_y - pos[1], target_z - pos[2]])

#     if dist < 0.1 and current_waypoint < len(waypoints) - 1:
#         current_waypoint += 1

#     z = pos[2]
#     vz = p.getBaseVelocity(targid)[0][2]

#     # === PD Control for Altitude ===
#     z_error = target_z - z
#     Kp_z = 7.0      # Position gain
#     Kv_z = 3.0      # Velocity damping

#     thrust = 9.81 + (Kp_z * z_error) - (Kv_z * vz)
#     thrust = np.clip(thrust, -1.0, 1.5)

#     print(f"Step: {step} | Z: {z:.2f} | Target Z: {target_z:.2f} | vz: {vz:.2f} | Thrust: {thrust:.2f}")

#     # === XY Position Control ===
#     error_x = target_x - pos[0]
#     error_y = target_y - pos[1]
#     d_error_x = error_x - prev_error_x
#     d_error_y = error_y - prev_error_y

#     x_force = 2.0 * error_x
#     y_force = 2.0 * error_y

#     prev_error_x = error_x
#     prev_error_y = error_y

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

#     # === Apply Forces and Torques ===
#     p.applyExternalForce(targid, -1, [x_force, y_force, thrust], [0, 0, 0], p.LINK_FRAME)
#     p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, yaw_torque], p.LINK_FRAME)

#     # === Log Data ===
#     altitude_log.append(z)
#     thrust_log.append(thrust)
#     x_log.append(pos[0])
#     y_log.append(pos[1])

#     p.stepSimulation()
#     time.sleep(0.01)








# **Results**
plt.figure(figsize=(12, 6))
plt.subplot(3, 1, 1)
plt.plot(altitude_log, label="Altitude (m)")
plt.axhline(y=max_altitude, color="r", linestyle="--", label="Max Altitude")
plt.legend()
plt.ylabel("Altitude (m)")
plt.title("Drone Altitude Over Time")

plt.subplot(3, 1, 2)
plt.plot(thrust_log, label="Thrust (N)", color="g")
plt.legend()
plt.ylabel("Thrust (N)")
plt.title("Thrust Over Time")

plt.subplot(3, 1, 3)
plt.plot(x_log, y_log, label="Drone Path")
plt.scatter([1.5, 3.0], [1.25, 1.25], color="r", marker="x", label="Waypoints")
plt.scatter(3.0, 1.25, color="g", marker="o", label="Landing Mark")
plt.legend()
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Drone X-Y Movement")

plt.show()

fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

ax.plot(x_log, y_log, altitude_log, label='Drone Path')
way_x = [wp[0] for wp in waypoints]
way_y = [wp[1] for wp in waypoints]
way_z = [wp[2] for wp in waypoints]

ax.scatter(way_x, way_y, way_z, color='red', label='Waypoints', marker='x', s=50)
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("3D Drone Trajectory")
ax.legend()

plt.tight_layout()
plt.show()







