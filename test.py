import pybullet as p
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

planeId = p.loadURDF("plane.urdf")

startPos = [0,0,0]
startOrientation = p.getQuaternionFromEuler([0,0,0])
f_path = "abb_irb120_support/urdf/irb120_3_58_macro.xacro"
arm = p.loadURDF(f_path, [0,0,0], startOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)

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
        p.setJointMotorControl2(arm, joint, p.POSITION_CONTROL, value)
        joint_state = p.getJointState(arm, joint)

        cp = None

        while np.abs(value - joint_state[0]) > .001:
            p.stepSimulation()
            time.sleep(1./240.)

            # Contact Points do not get reset after resetting the joint?
            cp = p.getContactPoints()
            if len(cp) > 0:
                for c in cp:
                    print(f"Link {c[3]} hit Link {c[4]}")
                for i in range(6):
                    p.resetJointState(arm, i, 0)
                break

            joint_state = p.getJointState(arm, joint)

            

# Move all joints
# for y in range(2):
#     for x in range(6):
#         time.sleep(1)
#         positions = np.zeros((1,9))[0]
#         positions[x] = 1
#         p.setJointMotorControlArray(arm, range(9), p.POSITION_CONTROL, positions)

#         j_states = p.getJointStates(arm, range(6))
#         thetas = [i[0] for i in j_states]

#         while np.abs(1 - thetas[x]) > .001:
#             p.stepSimulation()
#             time.sleep(1./240.)
#             j_states = p.getJointStates(arm, range(6))
#             thetas = [i[0] for i in j_states]
#             # print(np.round(thetas, 3))
#         time.sleep(1)
#         p.resetJointState(arm, x, 0)
#         p.stepSimulation()
        

p.disconnect()

