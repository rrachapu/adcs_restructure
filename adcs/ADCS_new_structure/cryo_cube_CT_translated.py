# returns transformation matrix given quaternion
import numpy as np
def quaternion2CTmatrix(quaternion):
    transform_matrix = np.zeros((3,3))
    transform_matrix[0][0] = 1 - 2*(quaternion[2]*quaternion[2] + quaternion[3]*quaternion[3])
    transform_matrix[0][1] = 2*(quaternion[1]*quaternion[2] + quaternion[0]*quaternion[3])
    transform_matrix[0][2] = 2*(quaternion[1]*quaternion[3] - quaternion[0]*quaternion[2])
    transform_matrix[1][0] = 2*(quaternion[1]*quaternion[2] - quaternion[0]*quaternion[3])
    transform_matrix[1][1] = 1 - 2*(quaternion[1]*quaternion[1] + quaternion[3]*quaternion[3])
    transform_matrix[1][2] = 2*(quaternion[2]*quaternion[3] + quaternion[0]*quaternion[1])
    transform_matrix[2][0] = 2*(quaternion[1]*quaternion[3] + quaternion[0]*quaternion[2])
    transform_matrix[2][1] = 2*(quaternion[2]*quaternion[3] - quaternion[0]*quaternion[1])
    transform_matrix[2][2] = 1 - 2*(quaternion[1]*quaternion[1] + quaternion[2]*quaternion[2])

    return transform_matrix

def T_dart(R, V):
    H = np.zeros(3)
    ih = np.zeros(3)
    ir = np.zeros(3)
    iv = np.zeros(3)

    r = np.linalg.norm(R)

    H = np.cross(R,V)
    
    h = np.linalg.norm(H)

    ih = (1/h)*H
    ir = (1/r)*R
    iv = np.cross(ih,ir)
    iv = (1/np.linalg.norm(iv))*iv

    T_dart = np.zeros((3,3))
    ang_vel_dart = np.zeros(3)

    T_dart[0][0] = ir[0]
    T_dart[0][1] = ir[1]
    T_dart[0][2] = ir[2]
    T_dart[1][0] = ih[0]
    T_dart[1][1] = ih[1]
    T_dart[1][2] = ih[2]
    T_dart[2][0] = -iv[0]
    T_dart[2][1] = -iv[1]
    T_dart[2][2] = -iv[2]

    ang_vel_dart[0] = 0.0
    ang_vel_dart[1] = h/(r*r)
    ang_vel_dart[2] = 0

    return T_dart, ang_vel_dart




        
