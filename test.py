import pybullet as p
import time
import pybullet_data
import numpy as np
from forward_k import *

# Set the joint values taking into account the offset
def set_arm_state(th_list):
    th_list[1] += np.pi/2
    for i in range(6):
        p.resetJointState(arm, i, th_list[i])

# Get the current joint values taking into account the offset
def get_arm_state():
    all_joints = [i[0] for i in p.getJointStates(arm, range(6))]
    all_joints[1] -= np.pi/2
    return all_joints

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

startOrientation = p.getQuaternionFromEuler([0,0,0])
f_path = "irb120.urdf"
arm = p.loadURDF(f_path, [0,0,.25], startOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)

set_arm_state(np.zeros((1,6))[0])
p.stepSimulation()

while True:
    command = input("Command: ")
    if command is "?":
        print("Available Commands:")
        print("\t?: Help menu")
        print("\tjoint_number:angle")
        print("\tq:Exit")
    elif command[0] is 'q':
        break
    else:
        joint = int(command[0])
        value = float(command[2:])
        all_joints = get_arm_state()
        all_joints[joint] = value
        set_arm_state(all_joints)
        p.stepSimulation()

        cp = p.getContactPoints()
        if len(cp) > 0:
            print("There were collisions")

        all_joints = [i[0] for i in p.getJointStates(arm, range(6))]
        H = dh_fwdK(all_joints)
        print(f"The joint values:\n {all_joints}\n result in the transform:\n {np.round(H, 3)}")  

p.disconnect()



