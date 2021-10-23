import numpy as np

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
    dh_table = np.array([
                        [0,     .29,    -1*pi/2,    0],
                        [.270,  0,      0,          pi/2],
                        [.07,   0,      -1*pi/2,    0],
                        [0,     .302,   pi/2,       0],
                        [0,     0,      -1*pi/2,    0],
                        [0,     .072,   0,          0]])

    H = np.eye(4)
    for i in range(6):
        a = dh_table[i][0]
        d = dh_table[i][1]
        alpha = dh_table[i][2]
        theta = th_list[i] - dh_table[i][3]
        H = H @ dhTransform(a, d, alpha, theta)
        
    return H
