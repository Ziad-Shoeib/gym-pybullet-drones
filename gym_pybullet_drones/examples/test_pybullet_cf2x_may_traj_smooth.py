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



# ### Trial 3: Adding More waypoints -- Position Trajectory
# # **PID gains (Tuned for altitude stability)**
# Kp_thrust = 0.4
# Kd_thrust = 2.0
# Ki_thrust = 15.2

# Kp_attitude = 2.0  
# Kd_attitude = 1.0  

# Kp_xy = 0.3  
# Kd_xy = 0.1  

# Kp_yaw = 1.0  # **New yaw control gain**

# # ###**Waypoints (Max altitude = 1.5m)**
# # waypoints = [
# #     [0.0, 0.0, 0.5],    # Takeoff straight up
# #     [0.5, 0.5, 1.0],    # Start moving forward & up
# #     [1.0, 1.25, 1.5],   # Approach gap entrance at center
# #     [1.5, 1.25, 1.5],   # Midway inside the gap
# #     [2.0, 1.25, 1.5],   # Exit the gap
# #     [2.5, 1.25, 1.3],   # Move forward
# #     [3.0, 1.25, 1.0],   # Approach landing
# #     [3.0, 1.25, 0.5],   # Descend partway
# #     [3.0, 1.25, 0.1],   # Final land
# # ]

# ## Bazier Position Trajectory
# df = pd.read_csv("bezier_position.csv", header=0)  # Skip header row
# df = df.astype(float)  # Convert to float
# waypoints = df.values.tolist()


# current_waypoint = 0

# # **Set Maximum Altitude**
# max_altitude = 1.0  
# min_altitude = 0.1  

# # **Thrust Limits**
# thrust = 9.8  
# max_thrust = 9.9  
# min_thrust = 0.01  
# thrust_change_limit = 0.8  

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

# for step in range(15000):
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

#     if dist < 0.1 and current_waypoint < len(waypoints) - 1:
#         current_waypoint += 1  

#     #### ✅ **Altitude Control (Thrust)**
#     error_z = target_z - pos[2]
#     d_error_z = error_z - prev_error_z
#     #integral_error_z = max(-0.05, min(0.05, error_z))  
#     integral_error_z += error_z  # accumulate
#     integral_error_z = np.clip(integral_error_z, -0.5, 0.5)

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
#     x_force = 2.0 * error_x  
#     y_force = 2.0 * error_y  

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

# fig = plt.figure(figsize=(10, 7))
# ax = fig.add_subplot(111, projection='3d')

# ax.plot(x_log, y_log, altitude_log, label='Drone Path')
# way_x = [wp[0] for wp in waypoints]
# way_y = [wp[1] for wp in waypoints]
# way_z = [wp[2] for wp in waypoints]

# ax.scatter(way_x, way_y, way_z, color='red', label='Waypoints', marker='x', s=50)
# ax.set_xlabel("X (m)")
# ax.set_ylabel("Y (m)")
# ax.set_zlabel("Z (m)")
# ax.set_title("3D Drone Trajectory")
# ax.legend()

# plt.tight_layout()
# plt.show()





############################## VelocityTrajectories ##############################

# # === VelocityTrajectories ===
# waypoints = pd.read_csv("bezier_position.csv", header=0).astype(float).values.tolist()
# velocity_refs = pd.read_csv("bezier_velocity.csv", header=0).astype(float).values.tolist()

# assert len(waypoints) == len(velocity_refs), "Mismatch between position and velocity lengths!"

# # === Controller Gains ===
# Kp_thrust = 0.4
# Kd_thrust = 2.0
# Ki_thrust = 15.2

# Kp_attitude = 2.0
# Kd_attitude = 1.0

# Kp_vel = 5.0  # velocity PD
# Kd_vel = 1.5

# Kp_yaw = 1.5

# # === Altitude & Thrust Parameters ===
# max_altitude = 1.5
# min_altitude = 0.1
# thrust = 9.8
# max_thrust = 12.9
# min_thrust = 0.01
# thrust_change_limit = 2.8

# # === Init Logs and Errors ===
# prev_error_z = 0
# integral_error_z = 0
# prev_error_roll = 0
# prev_error_pitch = 0
# prev_error_yaw = 0

# altitude_log = []
# thrust_log = []
# x_log = []
# y_log = []

# # === Main Loop ===
# for step in range(len(waypoints)):
#     p.resetBaseVelocity(targid, [0, 0, 0], [0, 0, 0])
#     pos, orn = p.getBasePositionAndOrientation(targid)
#     roll, pitch, yaw = p.getEulerFromQuaternion(orn)
#     lin_vel, _ = p.getBaseVelocity(targid)
#     vx, vy, vz = lin_vel

#     # Reference
#     target_x, target_y, target_z = waypoints[step]
#     vx_des, vy_des, vz_des = velocity_refs[step]

#     # Altitude Control (PD + I)
#     error_z = target_z - pos[2]
#     d_error_z = error_z - prev_error_z
#     integral_error_z += error_z
#     integral_error_z = np.clip(integral_error_z, -0.5, 0.5)
#     target_thrust = 9.8 + (Kp_thrust * error_z) + (Kd_thrust * d_error_z) + (Ki_thrust * integral_error_z)
#     thrust += np.clip(target_thrust - thrust, -thrust_change_limit, thrust_change_limit)
#     thrust = np.clip(thrust, min_thrust, max_thrust)
#     prev_error_z = error_z

#     # XY Velocity Control
#     error_vx = vx_des - vx
#     error_vy = vy_des - vy
#     x_force = Kp_vel * error_vx - Kd_vel * vx
#     y_force = Kp_vel * error_vy - Kd_vel * vy

#     # Yaw Control
#     error_yaw = -yaw
#     d_error_yaw = error_yaw - prev_error_yaw
#     yaw_torque = Kp_yaw * error_yaw + 0.5 * d_error_yaw
#     prev_error_yaw = error_yaw

#     # Attitude Stabilization
#     error_roll = -roll
#     error_pitch = -pitch
#     d_error_roll = error_roll - prev_error_roll
#     d_error_pitch = error_pitch - prev_error_pitch
#     roll_torque = Kp_attitude * error_roll + Kd_attitude * d_error_roll
#     pitch_torque = Kp_attitude * error_pitch + Kd_attitude * d_error_pitch
#     prev_error_roll = error_roll
#     prev_error_pitch = error_pitch

#     # Apply Forces
#     p.applyExternalForce(targid, -1, [x_force, y_force, thrust], [0, 0, 0], p.LINK_FRAME)
#     p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, yaw_torque], p.LINK_FRAME)

#     # Log
#     altitude_log.append(pos[2])
#     thrust_log.append(thrust)
#     x_log.append(pos[0])
#     y_log.append(pos[1])

#     print(f"Step {step}: Z={pos[2]:.2f} | Thrust={thrust:.2f} | vx={vx:.2f}/{vx_des:.2f}")

#     p.stepSimulation()
#     time.sleep(0.01)






# # === Plotting ===
# plt.figure(figsize=(12, 6))
# plt.subplot(3, 1, 1)
# plt.plot(altitude_log, label="Altitude")
# plt.axhline(y=max_altitude, color='r', linestyle='--', label="Max Altitude")
# plt.legend(); plt.ylabel("Z (m)"); plt.title("Altitude")

# plt.subplot(3, 1, 2)
# plt.plot(thrust_log, label="Thrust", color='g')
# plt.legend(); plt.ylabel("N"); plt.title("Thrust")

# plt.subplot(3, 1, 3)
# plt.plot(x_log, y_log, label="XY Path")
# plt.xlabel("X (m)"); plt.ylabel("Y (m)")
# plt.title("2D Path"); plt.legend()


# plt.subplot(3, 1, 3)
# plt.plot(x_log, y_log, label="Actual XY Path", linewidth=2)
# way_x = [wp[0] for wp in waypoints]
# way_y = [wp[1] for wp in waypoints]
# plt.plot(way_x, way_y, '--', label="Reference XY Path", color='orange')
# plt.xlabel("X (m)")
# plt.ylabel("Y (m)")
# plt.title("XY Path Comparison")
# plt.legend()


# fig = plt.figure(figsize=(10, 7))
# ax = fig.add_subplot(111, projection='3d')
# ax.plot(x_log, y_log, altitude_log, label='Actual Trajectory', linewidth=2)
# ax.plot(way_x, way_y, [wp[2] for wp in waypoints], '--', label='Reference Trajectory', color='orange')
# ax.set_xlabel('X (m)')
# ax.set_ylabel('Y (m)')
# ax.set_zlabel('Z (m)')
# ax.set_title("3D Trajectory Comparison")
# ax.legend()

# plt.tight_layout()
# plt.show()



### Velocity Hybrid Control - Just Worked as good as the above but better with the z-hight start - best method till now###

# === Load trajectory reference ===
position_refs = pd.read_csv("bezier_position.csv", header=0).astype(float).values.tolist()
velocity_refs = pd.read_csv("bezier_velocity.csv", header=0).astype(float).values.tolist()
assert len(position_refs) == len(velocity_refs)

# # Make the last 5 velocity waypoints descend gently
# for i in range(1, 300):  # Last 5 steps
#     idx = -i
#     velocity_refs[idx][2] = -10.9  # Set vz = -0.3 m/s (downward)

# === Gains ===
Kp_xy = 10.0   # Increase to pull drone harder toward the path
Kd_xy = 5.0    # Increase to dampen overshoot and reduce lag
Kp_thrust = 10.0   # Stronger response to height errors
Kd_thrust = 2.5
Ki_thrust = 18.0  # Helps reduce long-term offset
Kp_attitude = 2.0
Kd_attitude = 1.0
Kp_yaw = 1.0

# === Altitude / Thrust Settings ===
thrust = 9.8
max_thrust = 18.9   # Play with it till 20 to get better results
min_thrust = -20.0
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
    if pos[2] >= takeoff_z - 0.05:
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


    #  # === Altitude Control (Improved Descent Handling) ===
    # error_z = ref_pos[2] - pos[2]
    # d_error_z = error_z - prev_error_z
    # integral_error_z += error_z
    # integral_error_z = np.clip(integral_error_z, -0.5, 0.5)

    # # === Final descent trigger if reference Z is low
    # final_landing = ref_pos[2] < 0.2 and step > len(position_refs) - 20

    # if final_landing:
    #     thrust = 5.0  # Enforce descent
    # else:
    #     target_thrust = (
    #         9.81 +
    #         Kp_thrust * error_z +
    #         Kd_thrust * d_error_z +
    #         Ki_thrust * integral_error_z
    #     )
    #     thrust += np.clip(target_thrust - thrust, -thrust_change_limit, thrust_change_limit)
    #     thrust = np.clip(thrust, min_thrust, max_thrust)

    # prev_error_z = error_z



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

    # ## === Forced Landing - Worked Good but bad logically === ##
    # # Compute current distance to final landing point (XY only)
    # landing_xy = np.array(position_refs[-1][:2])
    # drone_xy = pos_arr[:2]
    # xy_dist = np.linalg.norm(drone_xy - landing_xy)

    # # If close enough to landing point, begin descent by commanding negative thrust
    # if xy_dist < 0.1:  # Adjust threshold if needed
    #     # You may apply a small downward force or negative thrust
    #     landing_thrust = -40.0  # Try different values like -1.0, -3.0
    #     total_thrust = landing_thrust
    # else:
    #     total_thrust = thrust  # normal control

    # # Apply control
    # p.applyExternalForce(targid, -1, [x_force, y_force, total_thrust], [0, 0, 0], p.LINK_FRAME)
    # p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, yaw_torque], p.LINK_FRAME)

    # === Log ===
    altitude_log.append(pos[2])
    thrust_log.append(thrust)
    x_log.append(pos[0])
    y_log.append(pos[1])
    z_log.append(pos[2])

    print(f"Step {step} | Z: {pos[2]:.2f} | Thrust: {thrust:.2f} | Error XY: {error_pos}")

    p.stepSimulation()
    time.sleep(0.01)

# # === Final Descent (Failsafe) ===
# for _ in range(400):
#     p.resetBaseVelocity(targid, [0, 0, 0], [0, 0, 0])
#     p.applyExternalForce(targid, -1, [0, 0, -5.0], [0, 0, 0], p.LINK_FRAME)
#     p.stepSimulation()
#     time.sleep(0.01)

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