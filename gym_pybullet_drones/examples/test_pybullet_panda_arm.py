# Following This Tutorial: https://www.youtube.com/watch?v=kZxPaGdoSJY

import numpy as np
import pybullet as p
import pybullet_data
import time # as simulation time needs a delay

p.connect(p.GUI)    # if we want no visuals p.connect(p.DIRECT)
p.resetSimulation() 
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)  # (x, y, z)
p.setRealTimeSimulation(0)


## Load Assets
p.loadURDF("plane.urdf", [0 , 0 ,0 ] , [0 , 0 , 0 , 1 ])    # Ground Plane (Not to have everything fall) (position_space(x,y,z), orientation(quaternion))
targid = p.loadURDF("franka_panda/panda.urdf", [0 , 0 , 0 ] , [0 , 0 , 0 , 1 ] , useFixedBase = True)   # (file_name, position_space(x,y,z), orientation(quaternion) ) (FixBase to make the base fixed)
obj_of_focus = targid   # To Focus on this specific asset

print(p.getNumJoints(targid)) #to get number of joints of the urdf = 12
for i in range(p.getNumJoints(targid)): # If we wanted the info of each joint
    print(p.getJointInfo(targid , i))

jointid = 4 # The joint number we want to get its value 
jlower = p.getJointInfo(targid, jointid ) [8]   # Minimum angle that the joint can go (in index 8)
jupper = p.getJointInfo(targid, jointid ) [9]   # Maximum angle that the joint can go (in index 9)

for step in range(500):
    joint_two_targ = np.random.uniform(jlower , jupper) # Just Sampling new target positions
    joint_four_targ = np.random.uniform(jlower , jupper)
    p.setJointMotorControlArray(targid, [2 , 4 ] , p.POSITION_CONTROL, targetPositions = [ joint_two_targ , joint_four_targ ])  # Apply forces to change the position
    #print(p.getJointStates(targid , [ 2 , 4 ]))
    #print(p.getLinkStates(targid , [ 2 , 4 ]))
    focus_position , focus_orientation = p.getBasePositionAndOrientation(targid)    # Determine the position and orientation of our target_id   
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=0, cameraPitch=-40, cameraTargetPosition = focus_position) # To Focus the camera on the focus position
    p.stepSimulation()
    time.sleep(.01)

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


       
