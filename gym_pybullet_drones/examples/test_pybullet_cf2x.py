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

####### Force Control #######

# Trial step 1
# for step in range(5000):
    
#     '''
#     p.applyExternalForce() applies a force to the drone.
#     targid → The ID of the drone in PyBullet.
    
#     -1 → Applies force to the whole drone body (base link).
    
#     [0, 0, thrust] → The force vector:
#     0 (X), 0 (Y), thrust (Z), meaning force is applied only upwards.
    
#     [0, 0, 0] → The position where the force is applied (center of mass).
#     p.WORLD_FRAME → The force is applied in the world coordinate frame, not the local drone frame.
    
#     '''
    
#     # Frist trial, but some times flip but flies up only - if p.LINK_FRAME deviates also
#     # thrust = 0.5  # Adjust this value to test different forces
#     # p.applyExternalForce(targid, -1,     [0, 0, thrust], [0, 0, 0], p.LINK_FRAME)  
    
#     # # Trial 2, doesn't flip but deviates to left - but also flips if WORLD_FRAME
#     # thrust = 0.2
#     # motor_forces = [0.5 * thrust] * 4  # Apply balanced force on all four props

#     # for i in range(4):
#     #     p.applyExternalForce(targid, i, [0, 0, motor_forces[i]], [0, 0, 0], p.LINK_FRAME)

#     # Get and print drone position to verify motion
#     pos, orn = p.getBasePositionAndOrientation(targid)
#     print("Drone Position:", pos)

#     p.stepSimulation()
#     time.sleep(0.01)

# # # Trial step 2 - Reduce Thrust Value Gradually - Deviates but not too hard
# thrust = 0
# for step in range(5000):
#     thrust += 0.001  # Gradual increase to avoid sudden jumps
#     thrust = min(thrust, 2.0)  # Cap max thrust

#     p.applyExternalForce(targid, -1, [0, 0, thrust], [0, 0, 0], p.LINK_FRAME)
#     pos, orn = p.getBasePositionAndOrientation(targid)
#     print("Drone Position:", pos)
#     p.stepSimulation()
#     time.sleep(0.01)

## Trial 3 - A simple Proportional Controller (PD-controller) can adjust thrust to prevent tilting: Also sudden Jump
# target_height = 0.2  # Desired altitude
# Kp = 0.3  # Proportional gain
# Kd = 0.3  # Damping term (derivative gain)
# prev_error = 0

# for step in range(5000):
#     pos, _ = p.getBasePositionAndOrientation(targid)
#     error = target_height - pos[2]  # Height difference
#     error_derivative = error - prev_error  # Rate of change of error

#     thrust = max(0, min(9.8 + Kp * error + Kd * error_derivative, 15))  # PD Control
#     prev_error = error  # Store error for next iteration

#     p.applyExternalForce(targid, -1, [0, 0, thrust], [0, 0, 0], p.LINK_FRAME)
#     p.stepSimulation()
#     time.sleep(0.01)


####### Torque Only Control #######

# # Trial 1 - It is a strange behaviour - Since no force is applied, the drone will fall due to gravity unless thrust is added.
# # Torque values (adjust for testing)
# roll_torque = 0.2   # Rotate around X-axis
# pitch_torque = -0.2  # Rotate around Y-axis
# yaw_torque = 0.0    # Rotate around Z-axis (spin drone)

# for step in range(5000):
#     # Apply torques in the drone's local frame (LINK_FRAME)
#     p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, yaw_torque], p.LINK_FRAME)
    
#     pos, orn = p.getBasePositionAndOrientation(targid)
#     print("Drone Position:", pos)
#     print("Drone Orientation:", orn)
    
#     p.stepSimulation()
#     time.sleep(0.01)




# ####### Force and Torque Control - Hover #######

# ## Trial 1: This works greatly as the drone only moves in z without deviation and it is not flipping - but target hight is wrong 
# # Actually after plotting, it takes a long time but reaches close at the end - 23 sec
# # # PD gains (tune these)
# Kp_thrust = 50.0  # Proportional gain for altitude
# Kd_thrust = 60.0  # Derivative gain for altitude

# Kp_attitude = 3.0  # Proportional gain for roll/pitch correction
# Kd_attitude = 1.0  # Derivative gain for roll/pitch correction

# # Desired drone state
# target_height = 1.0  # Desired altitude
# prev_error_z = 0
# prev_error_roll = 0
# prev_error_pitch = 0

# ##**Logging variables**
# altitude_log = []
# thrust_log = []

# for step in range(2300):
#     # Get drone position and orientation
#     pos, orn = p.getBasePositionAndOrientation(targid)
#     roll, pitch, yaw = p.getEulerFromQuaternion(orn)

#     #### ✅ Altitude PID Control (Thrust) ####
#     error_z = target_height - pos[2]  # Height error
#     d_error_z = error_z - prev_error_z  # Change in error

#     thrust = 9.8 + (Kp_thrust * error_z) + (Kd_thrust * d_error_z)  # PD control
#     thrust = max(0, min(thrust, 20))  # Limit thrust

#     prev_error_z = error_z  # Store error for next iteration

#     #### ✅ Attitude PID Control (Roll & Pitch Torque) ####
#     error_roll = -roll  # We want roll = 0 (level)
#     error_pitch = -pitch  # We want pitch = 0 (level)

#     d_error_roll = error_roll - prev_error_roll
#     d_error_pitch = error_pitch - prev_error_pitch

#     roll_torque = Kp_attitude * error_roll + Kd_attitude * d_error_roll
#     pitch_torque = Kp_attitude * error_pitch + Kd_attitude * d_error_pitch

#     prev_error_roll = error_roll
#     prev_error_pitch = error_pitch

#     #### ✅ Apply Forces & Torques ####
#     p.applyExternalForce(targid, -1, [0, 0, thrust], [0, 0, 0], p.LINK_FRAME)  # Apply thrust
#     p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, 0], p.LINK_FRAME)  # Apply torque

#     pos, orn = p.getBasePositionAndOrientation(targid)
#     print("Drone Position:", pos)
#     print("Drone Orientation:", orn)

#     ##**Log altitude and thrust values**
#     altitude_log.append(pos[2])
#     thrust_log.append(thrust)


#     p.stepSimulation()
#     time.sleep(0.01)

# plt.figure(figsize=(10, 5))
# plt.subplot(2, 1, 1)
# plt.plot(altitude_log, label="Altitude (m)")
# plt.axhline(y=target_height, color="r", linestyle="--", label="Target Altitude")
# plt.legend()
# plt.ylabel("Altitude (m)")
# plt.title("Drone Altitude Over Time")

# plt.subplot(2, 1, 2)
# plt.plot(thrust_log, label="Thrust (N)", color="g")
# plt.legend()
# plt.xlabel("Time Steps")
# plt.ylabel("Thrust (N)")
# plt.title("Thrust Over Time")

# plt.show()    


# ## Trial 2 - PID - Terrible

# # Set control parameters (tune these values for better stability)
# # Tuned PID gains
# Kp_thrust = 1.2  # Reduce P gain to avoid aggressive corrections
# Kd_thrust = 4.5  # Increase D gain for better damping
# Ki_thrust = 0.005  # Small integral gain to eliminate steady-state error

# Kp_attitude = 3.0  # Attitude control gain (roll/pitch)
# Kd_attitude = 1.5  # Damping for roll/pitch

# # Target altitude
# target_height = 1.0
# prev_error_z = 0
# integral_error_z = 0  # Stores accumulated altitude error

# prev_error_roll = 0
# prev_error_pitch = 0

# # Thrust clamping & smooth changes
# thrust = 9.8
# max_thrust = 13  # Prevent excessive lift
# min_thrust = 7.5  # Prevent free fall
# thrust_change_limit = 0.05  # **NEW** - Prevents sudden thrust jumps

# # **Logging variables**
# altitude_log = []
# thrust_log = []

# for step in range(500):
#     # Get drone position and orientation
#     pos, orn = p.getBasePositionAndOrientation(targid)
#     roll, pitch, yaw = p.getEulerFromQuaternion(orn)

#     #### ✅ Altitude PID Control (Thrust) ####
#     error_z = target_height - pos[2]  # Compute height error
#     d_error_z = error_z - prev_error_z  # Compute rate of change of error
#     integral_error_z += error_z  # Accumulate error over time
#     integral_error_z = max(-0.2, min(integral_error_z, 0.2))  # Prevent integral windup

#     # **Compute thrust using PID (Proportional + Integral + Derivative)**
#     target_thrust = 9.8 + (Kp_thrust * error_z) + (Kd_thrust * d_error_z) + (Ki_thrust * integral_error_z)

#     # **Smoothly adjust thrust**
#     thrust += np.clip(target_thrust - thrust, -thrust_change_limit, thrust_change_limit)
#     thrust = max(min_thrust, min(thrust, max_thrust))  # Clamping thrust

#     prev_error_z = error_z  # Store error for next iteration

#     #### ✅ Attitude PD Control (Roll & Pitch Torque) ####
#     error_roll = -roll  
#     error_pitch = -pitch  

#     d_error_roll = error_roll - prev_error_roll
#     d_error_pitch = error_pitch - prev_error_pitch

#     roll_torque = Kp_attitude * error_roll + Kd_attitude * d_error_roll
#     pitch_torque = Kp_attitude * error_pitch + Kd_attitude * d_error_pitch

#     prev_error_roll = error_roll
#     prev_error_pitch = error_pitch

#     #### ✅ Apply Forces & Torques ####
#     p.applyExternalForce(targid, -1, [0, 0, thrust], [0, 0, 0], p.LINK_FRAME)  
#     p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, 0], p.LINK_FRAME)

#     # **Log altitude and thrust values**
#     altitude_log.append(pos[2])
#     thrust_log.append(thrust)

#     pos, orn = p.getBasePositionAndOrientation(targid)
#     print("Drone Position:", pos)
#     print("Drone Orientation:", orn)

#     p.stepSimulation()
#     time.sleep(0.01)


# plt.figure(figsize=(10, 5))
# plt.subplot(2, 1, 1)
# plt.plot(altitude_log, label="Altitude (m)")
# plt.axhline(y=target_height, color="r", linestyle="--", label="Target Altitude")
# plt.legend()
# plt.ylabel("Altitude (m)")
# plt.title("Drone Altitude Over Time")

# plt.subplot(2, 1, 2)
# plt.plot(thrust_log, label="Thrust (N)", color="g")
# plt.legend()
# plt.xlabel("Time Steps")
# plt.ylabel("Thrust (N)")
# plt.title("Thrust Over Time")

# plt.show()




####### Force and Torque Control - Hover+Forward #######

# ## Trial 1: Fail but the thrust changes values
# # **PID gains (Tuned for altitude stability)**
# Kp_thrust = 50.0  # **Smoother altitude correction**
# Kd_thrust = 60.0  # **Better damping**
# Ki_thrust = 0.00002  # **Very small integral gain**

# Kp_attitude = 2.0  
# Kd_attitude = 1.0  

# Kp_xy = 0.3  
# Kd_xy = 0.1  

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
# max_thrust = 9.9  # **Lower max thrust**
# min_thrust = 5.0  
# thrust_change_limit = 0.02  

# # **Initialize variables**
# prev_error_z = 0
# integral_error_z = 0  
# prev_error_x = 0
# prev_error_y = 0
# prev_error_roll = 0
# prev_error_pitch = 0

# # Logging
# altitude_log = []
# thrust_log = []
# x_log = []
# y_log = []

# for step in range(5000):
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
#     p.applyExternalTorque(targid, -1, [roll_torque, pitch_torque, 0], p.LINK_FRAME)

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

# # **Plot results**
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


    # pos, orn = p.getBasePositionAndOrientation(targid)
    # print("Drone Position:", pos)
    # print("Drone Orientation:", orn)

### Trial 2: Much better but the drone does not land exactly and overshoots a bit
# **PID gains (Tuned for altitude stability)**
Kp_thrust = 0.3  # **Smoother altitude correction**
Kd_thrust = 1.2  # **Better damping**
Ki_thrust = 0.00002  # **Very small integral gain**

Kp_attitude = 2.0  
Kd_attitude = 1.0  

Kp_xy = 0.3  
Kd_xy = 0.1  

Kp_yaw = 1.0  # **New yaw control gain**

# **Waypoints (Max altitude = 1.5m)**
waypoints = [[1.5, 1.25, 1.3],  
             [3.0, 1.25, 1.3],  
             [3.0, 1.25, 0.1]]  

current_waypoint = 0

# **Set Maximum Altitude**
max_altitude = 1.5  
min_altitude = 0.1  

# **Thrust Limits**
thrust = 9.8  
max_thrust = 9.9  
min_thrust = 5.0  
thrust_change_limit = 0.02  

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

for step in range(5000):
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

    if dist < 0.2 and current_waypoint < len(waypoints) - 1:
        current_waypoint += 1  

    #### ✅ **Altitude Control (Thrust)**
    error_z = target_z - pos[2]
    d_error_z = error_z - prev_error_z
    integral_error_z = max(-0.05, min(0.05, error_z))  

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
    x_force = 3 * error_x  
    y_force = 3 * error_y  

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











###########################################################################

########### Making a Class --> envs #############

# class ArmEnv():
#     def __init__(self):
#         self.state = self.init_state()
#         self.step_count = 0

#     def init_state(self):
#         p.connect(p.DIRECT)    # if we want no visuals p.connect(p.DIRECT)
#         p.resetSimulation() 
#         p.setAdditionalSearchPath(pybullet_data.getDataPath())
#         p.setGravity(0,0,-9.8)  # (x, y, z).
#         #p.setRealTimeSimulation(0)
#         self.pandaUid = p.loadURDF("franka_panda/panda.urdf", [0 , 0 , 0 ] , [0 , 0 , 0 , 1 ] , useFixedBase = True)   # (file_name, position_space(x,y,z), orientation(quaternion) ) (FixBase to make the base fixed)
#         p.loadURDF("plane.urdf", [0 , 0 ,0 ] , [0 , 0 , 0 , 1 ])    # Ground Plane
#         self.focus_pos , _ = p.getBasePositionAndOrientation(self.pandaUid)
#         self.jlower = p.getJointInfo(self.pandaUid, 4 ) [8]   # Minimum angle that the joint can go (in index 8)
#         self.jupper = p.getJointInfo(self.pandaUid, 4 ) [9]
#         p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition = self.focus_pos) # To Focus the camera on the focus position
#         finger_pos = p.getLinkState(self.pandaUid , 9 )[0]  # Last Link 9 --> [0] is the position
#         obs = np.array([ finger_pos ]).flatten()
#         return obs

#     def reset(self):
#         p.disconnect()
#         self.state = self.init_state()
#         self.step_count = 0

#     def step(self,new_pos):
#         self.step_count += 1
#         p.setJointMotorControlArray(self.pandaUid, [ 4 ] , p.POSITION_CONTROL, [new_pos] )   # Taking action on the forth joint
#         p.stepSimulation()
#         finger_pos = p.getLinkState(self.pandaUid , 9 )[0]

#         if (self.step_count >= 50 ) :   # Termination Condition
#             self.reset()
#             finger_pos = p.getLinkState(self.pandaUid , 9)[0]
#             obs = np.array([ finger_pos ]).flatten()
#             self.state = obs
#             reward = -1    # Arbitrary reward
#             done = True
#             return reward , done
        
#         obs = np.array([ finger_pos ]).flatten()
#         self.state = obs
#         done = False
#         reward = -1

#         return reward , done
        

# ########### Implementing the class (should be p.DIRECT to be useful, it give the states) #############

# env = ArmEnv()
# for step in range(500):
#     new_pos = np.random.uniform(env.jlower , env.jupper)
#     a , b = env.step(new_pos)
#     print(env.state)
#     p.stepSimulation()


       
