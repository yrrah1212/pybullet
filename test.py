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
arm = p.loadURDF(f_path, [0,0,0], startOrientation, useFixedBase=1)

for y in range(250):
    p.stepSimulation()
    time.sleep(1./240.)

# Move all joints
for y in range(2):
    for x in range(6):
        time.sleep(1)
        positions = np.zeros((1,9))[0]
        positions[x] = 1
        p.setJointMotorControlArray(arm, range(9), p.POSITION_CONTROL, positions)

        j_states = p.getJointStates(arm, range(6))
        thetas = [i[0] for i in j_states]

        while np.abs(1 - thetas[x]) > .001:
            p.stepSimulation()
            time.sleep(1./240.)
            j_states = p.getJointStates(arm, range(6))
            thetas = [i[0] for i in j_states]
            # print(np.round(thetas, 3))
        time.sleep(1)
        p.resetJointState(arm, x, 0)
        p.stepSimulation()
        

p.disconnect()

