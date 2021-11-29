import pybullet as p
import numpy as np
import pathlib

class Arm:
    def __init__(self):
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        # Path not in colab
        # f_path = f"{pathlib.Path().resolve()}/irb120.urdf"
        # Path in colab
        f_path = "/content/pybullet/IRB120_ENV/irb120_env/resources/irb120.urdf"
        self.arm = p.loadURDF(f_path, [0, 0, 0], startOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)
        # start in the zero angle config
        self.reset()

    def reset(self):
        th_list = [0, np.pi/2, 0, 0, 0, 0]
        for i in range(6):
            p.resetJointState(self.arm, i, th_list[i])

    def apply_action(self, th_list):
        p.resetJointState(self.arm, 0, th_list[0])
        p.resetJointState(self.arm, 1, th_list[1])

    def get_observations(self):
        joints = [p.getJointState(self.arm, i)[0] for i in range(6)]
        
        T = dh_fwdK(joints)

        return np.transpose(T[0:3, 3])


    def rotX(theta: float):
        s = np.sin(theta)
        c = np.cos(theta)
        r = np.array([[1,0,0],[0,c,-1*s],[0,s,c]])
        return r

    def rotZ(theta: float):
        s = np.sin(theta)
        c = np.cos(theta)
        r = np.array([[c,-1*s,0],[s,c,0],[0,0,1]])
        return r

    def rot2Quat(r):
        r11 = r[0,0]
        r22 = r[1,1]
        r33 = r[2,2]
        
        q0s = .25 * (1 + r11 + r22 + r33)
        q1s = .25 * (1 + r11 - r22 - r33)
        q2s = .25 * (1 - r11 + r22 - r33)
        q3s = .25 * (1 - r11 - r22 + r33)

        qs = np.array([q0s, q1s, q2s, q3s])

        max_idx = np.argmax(qs)

        if max_idx == 0:
            q0 = np.sqrt(q0s)
            
            q1 = .25*(r[2][1] - r[1][2]) / q0
            q2 = .25*(r[0][2] - r[2][0]) / q0
            q3 = .25*(r[1][0] - r[0][1]) / q0
        elif max_idx == 1:
            q1 = np.sqrt(q1s)
            
            q0 = .25*(r[2][1] - r[1][2]) / q1
            q2 = .25*(r[0][1] + r[1][0]) / q1
            q3 = .25*(r[0][2] + r[2][0]) / q1
        elif max_idx == 2:
            q2 = np.sqrt(q2s)
            
            q0 = .25*(r[0][2] - r[2][0]) / q2
            q1 = .25*(r[0][1] + r[1][0]) / q2
            q3 = .25*(r[1][2] + r[2][1]) / q2
        
        elif max_idx == 3:
            q3 = np.sqrt(q3s)
            
            q0 = .25*(r[1][0] - r[0][1]) / q3
            q1 = .25*(r[0][2] + r[2][0]) / q3
            q2 = .25*(r[1][2] + r[2][1]) / q3

        return [q0, q1, q2, q3]


    def dhTransform(a: float, d: float, alpha: float, theta: float):
        r1 = np.vstack((rotZ(theta), np.array([0,0,0])))
        d1 = np.array([[0],[0],[d],[1]])
        t1 = np.concatenate((r1, d1), axis=1)

        r2 = np.vstack((rotX(alpha), np.array([0,0,0])))
        d2 = np.array([[a],[0],[0],[1]])
        t2 = np.concatenate((r2, d2), axis=1)

        return t1@t2

    def dh_fwdK(th_list: list):
        pi = np.pi
        dh = np.array([
                            [0,     .29,    -1*pi/2,    0],
                            [.270,  0,      0,          pi/2],
                            [.07,   0,      -1*pi/2,    0],
                            [0,     .302,   pi/2,       0],
                            [0,     0,      -1*pi/2,    0],
                            [0,     .072,   0,          0]])

        H = np.eye(4)
        for i in range(6):
            H = H @ dhTransform(dh[i][0], dh[i][1], dh[i][2], th_list[i] - dh[i][3])
            
        return H
