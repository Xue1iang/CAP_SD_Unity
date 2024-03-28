#!/usr/bin/env python3
import tf
import rospy
import math
import tf
import numpy as np
from tf import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import geometry_msgs.msg
import datetime
import time
from numpy.linalg import inv
from scipy.spatial.transform import Rotation as R
import sys
import numpy.matlib as npm
import numpy
import quaternion
import pandas as pd
import scipy
from sympy import symbols, Eq, solve, tan, pi, sqrt, nsolve
from scipy.optimize import fsolve

# def qt2H(r, t):
#     # quaternion and translation to homogenous transformation matrix
#     # r is (4,) in [x,y,z,w] order
#     # t is (3,)
#     q = R.from_quat(r)
#     rotm = np.array(q.as_matrix())
#     H = ([[rotm[0,0], rotm[0,1], rotm[0,2], t[0]],
#           [rotm[1,0], rotm[1,1], rotm[1,2], t[1]],
#           [rotm[2,0], rotm[2,1], rotm[2,2], t[2]],
#           [0,          0,         0,         1   ]])

#     return H

# def qt2H(r, t):
#     # quaternion and translation to homogenous transformation matrix
#     # r is (4,) in [x,y,z,w] order
#     # t is (3,)
#     q = R.from_quat(r)
#     rotm = np.array(q.as_matrix())
#     rotm_ = np.array([[rotm[0,0], rotm[0,1], rotm[0,2], 0],
#               [rotm[1,0], rotm[1,1], rotm[1,2], 0],
#               [rotm[2,0], rotm[2,1], rotm[2,2], 0],
#               [0,         0,         0,         1]])

#     t_ = np.array([[1,0,0,t[0]],
#                    [0,1,0,t[1]],
#                    [0,0,1,t[2]],
#                    [0,0,0,1]])
#     H = np.matmul(rotm_, t_)
#     return H

def qt2H(r, t):
    # quaternion and translation to homogenous transformation matrix
    # r is (4,) in [x,y,z,w] order
    # t is (3,)
    # q = R.from_quat(r)
    # rotm = np.array(q.as_dcm())
    q = R.from_quat(r)
    rotm = np.array(q.as_matrix()) 

    rotm_ = np.array([[rotm[0,0], rotm[0,1], rotm[0,2], 0],
                      [rotm[1,0], rotm[1,1], rotm[1,2], 0],
                      [rotm[2,0], rotm[2,1], rotm[2,2], 0],
                      [0,         0,         0,         1]])
    t_ = np.array([[1,0,0,t[0]],
                   [0,1,0,t[1]],
                   [0,0,1,t[2]],
                   [0,0,0,1]])
    H = np.matmul(t_, rotm_)

    return H

# def qt2H_(r, t):
#     # quaternion and translation to homogenous transformation matrix
#     # r is (4,) in [x,y,z,w] order
#     # t is (3,)
#     q = R.from_quat(r)
#     rotm = np.array(q.as_matrix())
#     H = np.array([[rotm[0,0], rotm[0,1], rotm[0,2], t[0]],
#           [rotm[1,0], rotm[1,1], rotm[1,2], t[1]],
#           [rotm[2,0], rotm[2,1], rotm[2,2], t[2]],
#           [0,          0,         0,         1   ]])

#     return H

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    R = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ])
    return R
def transform_point(point, translation, rotation_matrix):
    """
    Transforms a point using translation and rotation.
    
    Parameters:
    - point: 3D coordinates of the point as a list or numpy array.
    - translation: 3D translation vector as a list or numpy array.
    - rotation_matrix: 3x3 rotation matrix as a numpy array.

    Returns:
    - Transformed 3D coordinates of the point as a numpy array.
    """
    
    # Apply translation
    translated_point = np.array(point) - np.array(translation)
    
    # Apply rotation
    transformed_point = np.matmul(rotation_matrix, translated_point)
    
    return transformed_point

def ds_cap(pressure_sens, t_W_B, r_W_B, t_B_C, r_B_C, t_C_P, r_C_P):
    # print(f"\npositioning_start: {pressure_sens}, {t_C_P}, {r_C_P}")

    # print(f"r_W_B is: {r_W_B}")
    H_W_B = qt2H(r_W_B, t_W_B)
    # print(f"H_W_B is: {H_W_B}")     
    H_B_C = qt2H(r_B_C, t_B_C)
    # print(f"H_B_C is: {H_B_C}") 
    # H_W_C = np.matmul(H_W_B, H_B_C)
    H_W_C = np.matmul(np.array(H_W_B), np.array(H_B_C))
    # print(f"H_W_C is: {H_W_C}") 
    x_W_C = H_W_C[0,3]
    y_W_C = H_W_C[1,3]
    z_W_C = H_W_C[2,3]

    # H_W_P = H_W_C * H_C_P
    
    H_C_P = qt2H(r_C_P, t_C_P)
    H_W_P = np.matmul(H_W_C, H_C_P)

    # print(f"positioning_mid  : {pressure_sens}, {t_C_P}, {r_C_P}")
    x_W_P = H_W_P[0,3]
    y_W_P = H_W_P[1,3]
    z_W_P = H_W_P[2,3]

    # tf_W_P = TransformBroadcaster()
    # tf_W_P.sendTransform((x_W_P, y_W_P, z_W_P), (0,0,0,1), rospy.Time.now(), 'PPPPPPPPPPPPPPPP', 'world')

    # if z_W_C == z_W_P:
        # z_W_C = 1000000000000 # Avoding being divided by 0
    # else:
    k = (pressure_sens - z_W_P) / (z_W_C - z_W_P)
    x_W_M = x_W_P + (x_W_C - x_W_P) * k
    y_W_M = y_W_P + (y_W_C - y_W_P) * k
    z_W_M = z_W_P + (z_W_C - z_W_P) * k
        
        # tf_W_M = TransformBroadcaster()
        # tf_W_M.sendTransform((x_W_M, y_W_M, z_W_M),  (0,0,0,1), rospy.Time.now(), 'MMMMMMMMMMMMMM', 'world')

    t_W_M = (x_W_M, y_W_M, z_W_M)
    # cap_offline_python = pd.DataFrame(t_W_M, columns=['x','y','z'])
    # cap_offline_python.to_csv('/home/xl/bag666/offlinepython1.csv', index=False)
    # print(f"positioning_end  : {pressure_sens}, {t_C_P}, {r_C_P}\n")
    # input_values = {
    #     'pressure_sens': pressure_sens,
    #     't_W_B': t_W_B,
    #     'r_W_B': r_W_B,
    #     't_B_C': t_B_C,
    #     'r_B_C': r_B_C,
    #     't_C_P': t_C_P,
    #     'r_C_P': r_C_P,
    #     't_W_M': t_W_M
    # }
    # return t_W_M, pressure_sens, t_W_B, r_W_B, t_C_P, r_C_P, input_values
    return t_W_M

def pnpcap_sys(t_W_B, r_W_B, t_B_C, r_B_C, t_C_M, r_C_M):
    # print(f"r_W_B is: {r_W_B}")
    H_W_B = qt2H(r_W_B, t_W_B)
    # print(f"H_W_B is: {H_W_B}")     
    H_B_C = qt2H(r_B_C, t_B_C)
    # print(f"H_B_C is: {H_B_C}") 
    # H_W_C = np.matmul(H_W_B, H_B_C)
    H_W_C = np.matmul(np.array(H_W_B), np.array(H_B_C))
    # print(f"H_W_C is: {H_W_C}") 

    H_C_M = qt2H(r_C_M, t_C_M)
    H_W_M = np.matmul(H_W_C, H_C_M)

    x_W_M = H_W_M[0,3]
    y_W_M = H_W_M[1,3]
    z_W_M = H_W_M[2,3]

    t_W_M = (x_W_M, y_W_M, z_W_M)

    return t_W_M




def dscap_wrong(pressure_sens, t_W_B, r_W_B, t_B_C, r_B_C, t_C_P, r_C_P,r_C_T):
    
    H_W_B = qt2H(r_W_B, t_W_B)
    t_B_C_ = np.array([t_B_C[0],t_B_C[1],t_B_C[2], 1])
    p_W_C = np.matmul(H_W_B, t_B_C_)

    x_W_C = p_W_C[0]
    y_W_C = p_W_C[1]
    z_W_C = p_W_C[2]

    H_B_C = qt2H(r_B_C, t_B_C)
    t_C_P_ = np.array([t_C_P[0],t_C_P[1],t_C_P[2],1])
    p_B_P = np.matmul(H_B_C, t_C_P_)

    p_W_P = np.matmul(H_W_B, p_B_P)
    x_W_P = p_W_P[0]
    y_W_P = p_W_P[1]
    z_W_P = p_W_P[2]


    k = (pressure_sens - z_W_P) / (z_W_C - z_W_P)
    x_W_M = x_W_P + (x_W_C - x_W_P) * k
    y_W_M = y_W_P + (y_W_C - y_W_P) * k
    z_W_M = z_W_P + (z_W_C - z_W_P) * k

    t_W_M = (x_W_M, y_W_M, z_W_M)

    q_c2t = np.quaternion(r_C_T[3], r_C_T[0], r_C_T[1], r_C_T[2])
    q_b2c = np.quaternion(r_B_C[3], r_B_C[0], r_B_C[1], r_B_C[2])
    q_w2b = np.quaternion(r_W_B[3], r_W_B[0], r_W_B[1], r_W_B[2])
    q = np.quaternion(0.7071, 0,0,0.7071)
    q_w2t = q_w2b * q_b2c * q_c2t * q
    # q_w2t = q * (q_c2t * (q_b2c * q_w2b))
    # q_w2t = q * q_c2t * q_b2c * q_w2b
    q_w2t = np.array([q_w2t.x, q_w2t.y, q_w2t.z, q_w2t.w])
    # print(q_w2t)
    q = R.from_quat([q_w2t[0], q_w2t[1], q_w2t[2], q_w2t[3]])
    eul_w2t = q.as_euler('zyx', degrees=True)
    yaw_w2t_radians = math.radians(eul_w2t[0])
    # return t_W_M
    return t_W_M, yaw_w2t_radians, q_w2t

def dscap_new(pressure_sens, t_W_B, r_W_B, t_B_C, r_B_C, t_C_P, r_C_P):
    # print(f"\npositioning_start: {pressure_sens}, {t_C_P}, {r_C_P}")
    # print(f"r_W_B is: {r_W_B}")
    r_W_B_ = quaternion_to_rotation_matrix(np.array([r_W_B[3],-r_W_B[0],-r_W_B[1],-r_W_B[2]]))
    # print(np.array([[t_W_B[0]],[t_W_B[1]],[t_W_B[2]]]))
    t_W_B = np.matmul(r_W_B_, -np.array([[t_W_B[0]],[t_W_B[1]],[t_W_B[2]]]))
    
    H_W_B = qt2H(np.array([-r_W_B[0], -r_W_B[1], -r_W_B[2], r_W_B[3]]), t_W_B)
    t_B_C_ = np.vstack([np.array([[t_B_C[0]],[t_B_C[1]],[t_B_C[2]]]),1])
    H_W_B = [[float(item) if isinstance(item, np.ndarray) else item for item in row] for row in H_W_B]
    # print(H_W_B)
    p_W_C = np.matmul(H_W_B, t_B_C_)
    # print(f"H_W_B is: {H_W_B}")     
    # H_B_C = qt2H(r_B_C, t_B_C)
    # print(f"H_B_C is: {H_B_C}") 
    # H_W_C = np.matmul(H_W_B, H_B_C)
    # H_W_C = np.array(H_B_C) @ np.array(H_W_B) 
    # print(f"H_W_C is: {H_W_C}") 
    x_W_C = p_W_C[0]
    y_W_C = p_W_C[1]
    z_W_C = p_W_C[2]

    # H_W_P = H_W_C * H_C_P
    
    H_B_C = qt2H(r_B_C, t_B_C)
    
    t_C_P_ = np.array([[t_C_P[0]], [t_C_P[1]], [t_C_P[2]]])

    # print(H_B_C)
    # H_W_P = np.matmul(H_C_P, H_W_C)
    p_P_B = np.matmul(H_B_C, np.vstack([t_C_P_,1]))
    p_P_W = np.matmul(H_W_B, p_P_B)
    # print(f"positioning_mid  : {pressure_sens}, {t_C_P}, {r_C_P}")
    x_W_P = p_P_W[0]
    y_W_P = p_P_W[1]
    z_W_P = p_P_W[2]

    # tf_W_P = TransformBroadcaster()
    # tf_W_P.sendTransform((x_W_P, y_W_P, z_W_P), (0,0,0,1), rospy.Time.now(), 'PPPPPPPPPPPPPPPP', 'world')

    # if z_W_C == z_W_P:
        # z_W_C = 1000000000000 # Avoding being divided by 0
    # else:
    k = (pressure_sens - z_W_P) / (z_W_C - z_W_P)
    x_W_M = x_W_P + (x_W_C - x_W_P) * k
    y_W_M = y_W_P + (y_W_C - y_W_P) * k
    z_W_M = z_W_P + (z_W_C - z_W_P) * k
        
        # tf_W_M = TransformBroadcaster()
        # tf_W_M.sendTransform((x_W_M, y_W_M, z_W_M),  (0,0,0,1), rospy.Time.now(), 'MMMMMMMMMMMMMM', 'world')

    t_W_M = (x_W_M, y_W_M, z_W_M)
    # cap_offline_python = pd.DataFrame(t_W_M, columns=['x','y','z'])
    # cap_offline_python.to_csv('/home/xl/bag666/offlinepython1.csv', index=False)
    # print(f"positioning_end  : {pressure_sens}, {t_C_P}, {r_C_P}\n")
    # input_values = {
    #     'pressure_sens': pressure_sens,
    #     't_W_B': t_W_B,
    #     'r_W_B': r_W_B,
    #     't_B_C': t_B_C,
    #     'r_B_C': r_B_C,
    #     't_C_P': t_C_P,
    #     'r_C_P': r_C_P,
    #     't_W_M': t_W_M
    # }
    # return t_W_M, pressure_sens, t_W_B, r_W_B, t_C_P, r_C_P, input_values
    return t_W_M

def ds_cap_pnpyaw(pressure_sens, t_W_B, r_W_B, t_B_C, r_B_C, t_C_P, r_C_P, r_C_T):
    # print(f"\npositioning_start: {pressure_sens}, {t_C_P}, {r_C_P}")

    # print(f"r_W_B is: {r_W_B}")
    H_W_B = qt2H(r_W_B, t_W_B)
    # print(f"H_W_B is: {H_W_B}")     
    H_B_C = qt2H(r_B_C, t_B_C)
    # print(f"H_B_C is: {H_B_C}") 
    # H_W_C = np.matmul(H_W_B, H_B_C)
    H_W_C = np.matmul(np.array(H_W_B), np.array(H_B_C))
    # print(f"H_W_C is: {H_W_C}") 
    x_W_C = H_W_C[0,3]
    y_W_C = H_W_C[1,3]
    z_W_C = H_W_C[2,3]

    # H_W_P = H_W_C * H_C_P
    
    H_C_P = qt2H(r_C_P, t_C_P)
    H_W_P = np.matmul(H_W_C, H_C_P)

    # print(f"positioning_mid  : {pressure_sens}, {t_C_P}, {r_C_P}")
    x_W_P = H_W_P[0,3]
    y_W_P = H_W_P[1,3]
    z_W_P = H_W_P[2,3]

    # tf_W_P = TransformBroadcaster()
    # tf_W_P.sendTransform((x_W_P, y_W_P, z_W_P), (0,0,0,1), rospy.Time.now(), 'PPPPPPPPPPPPPPPP', 'world')

    # if z_W_C == z_W_P:
        # z_W_C = 1000000000000 # Avoding being divided by 0
    # else:
    k = (pressure_sens - z_W_P) / (z_W_C - z_W_P)
    x_W_M = x_W_P + (x_W_C - x_W_P) * k
    y_W_M = y_W_P + (y_W_C - y_W_P) * k
    z_W_M = z_W_P + (z_W_C - z_W_P) * k
        
        # tf_W_M = TransformBroadcaster()
        # tf_W_M.sendTransform((x_W_M, y_W_M, z_W_M),  (0,0,0,1), rospy.Time.now(), 'MMMMMMMMMMMMMM', 'world')

    t_W_M = (x_W_M, y_W_M, z_W_M)

    q_c2t = np.quaternion(r_C_T[3], r_C_T[0], r_C_T[1], r_C_T[2])
    q_b2c = np.quaternion(r_B_C[3], r_B_C[0], r_B_C[1], r_B_C[2])
    q_w2b = np.quaternion(r_W_B[3], r_W_B[0], r_W_B[1], r_W_B[2])
    q = np.quaternion(0.7071, 0,0,-0.7071)
    # q_w2t = q_w2b * q_b2c * q_c2t
    
    q_w2t =  q_c2t * q_b2c * q_w2b * q
    q_w2t = np.array([q_w2t.x, q_w2t.y, q_w2t.z, q_w2t.w])
    # print(q_w2t)
    q = R.from_quat([q_w2t[0], q_w2t[1], q_w2t[2], q_w2t[3]])
    eul_w2t = q.as_euler('zyx', degrees=True)
    yaw_w2t_radians = math.radians(eul_w2t[0])
    

    # print(f"positioning_end  : {pressure_sens}, {t_C_P}, {r_C_P}\n")
    # return t_W_M, pressure_sens, t_W_B, r_W_B, t_C_P, r_C_P
    return t_W_M, yaw_w2t_radians, q_w2t

def unitq(axis,angle):
    # creat a unit quaternion
    axis_norm = axis/np.linalg.norm(axis)
    im = np.sin(angle/2) * axis_norm
    r = (im[0], im[1], im[2], np.cos(angle/2))
    return r



def spoof_cam(t_W_B, r_W_B, t_B_C, r_B_C, H_W_M):
    H_W_B = qt2H(r_W_B, t_W_B)           
    H_B_C = qt2H(r_B_C, t_B_C)
            
    H_W_C = np.matmul(H_W_B, H_B_C)
    
    # H_C_M = H_W_C_^-1 * H_W_M
    H_C_M_sim = np.matmul(inv(np.array(H_W_C)), np.array(H_W_M))

    # r_wc = R.from_matrix([[H_W_C[0,0], H_W_C[0,1],H_W_C[0,2]],
    #                               [H_W_C[1,0], H_W_C[1,1],H_W_C[1,2]],
    #                               [H_W_C[2,0], H_W_C[2,1],H_W_C[2,2]]])
    # r_W_C_forward = r_wc.as_quat()

    # r_cm = R.from_matrix([[H_C_M_sim[0,0], H_C_M_sim[0,1],H_C_M_sim[0,2]],
    #                       [H_C_M_sim[1,0], H_C_M_sim[1,1],H_C_M_sim[1,2]],
    #                       [H_C_M_sim[2,0], H_C_M_sim[2,1],H_C_M_sim[2,2]]])
    # r_C_M_sim = r_cm.as_quat()
            
    H_C_P = H_C_M_sim
    tx_C_P = H_C_P[0,3] / H_C_P[2,3]
    ty_C_P = H_C_P[1,3] / H_C_P[2,3]
    tz_C_P = H_C_P[2,3] / H_C_P[2,3]
    t_C_P = (tx_C_P, ty_C_P, tz_C_P)

    return t_C_P 

def transform_point(point, translation, rotation_matrix):
    """
    Transforms a point using translation and rotation.
    
    Parameters:
    - point: 3D coordinates of the point as a list or numpy array.
    - translation: 3D translation vector as a list or numpy array.
    - rotation_matrix: 3x3 rotation matrix as a numpy array.

    Returns:
    - Transformed 3D coordinates of the point as a numpy array.
    """
    # # Apply translation
    # translated_point = np.array(point) + np.array(translation)
    
    # # Apply rotation
    # transformed_point = np.matmul(rotation_matrix, translated_point)
    
    # Apply translation
    rotated_point = np.matmul(rotation_matrix, np.array(point))
    # print(rotated_point)
    transformed_point = np.array(rotated_point) - np.array(translation)
    
    # Apply rotation
    # transformed_point = np.matmul(rotation_matrix, translated_point)
    
    return transformed_point


import numpy as np

def quat2rotm(quat):
# This def converts quaternion to rotation matrix
# Input quaternion is in (w, x, y, z) order
    if len(quat) != 4:
        raise ValueError("Quaternion must have 4 elements.")

    rotm = np.zeros((3, 3))

    qnorm = np.linalg.norm(quat)
    if qnorm > 1:
        quat = quat / qnorm  # normalize

    # scalar (real) part
    q_0 = quat[0]
    # vector (imaginary) part
    q_1 = quat[1]
    q_2 = quat[2]
    q_3 = quat[3]

    # Direct assignment for Direction Cosine Matrix (DCM) 
    rotm[0, 0] = q_0*q_0 + q_1*q_1 - q_2*q_2 - q_3*q_3
    rotm[0, 1] = 2 * (q_1*q_2 - q_0*q_3)
    rotm[0, 2] = 2 * (q_0*q_2 + q_1*q_3)

    rotm[1, 0] = 2 * (q_0*q_3 + q_1*q_2)
    rotm[1, 1] = q_0*q_0 - q_1*q_1 + q_2*q_2 - q_3*q_3
    rotm[1, 2] = 2 * (q_2*q_3 - q_0*q_1)

    rotm[2, 0] = 2 * (q_1*q_3 - q_0*q_2)
    rotm[2, 1] = 2 * (q_0*q_1 + q_2*q_3)
    rotm[2, 2] = q_0*q_0 - q_1*q_1 - q_2*q_2 + q_3*q_3

    return rotm

 
def quaternion_multiply(Q0,Q1):
    """
    Multiplies two quaternions.
 
    Input
    :param Q0: A 4 element array containing the first quaternion (q01,q11,q21,q31) 
    :param Q1: A 4 element array containing the second quaternion (q02,q12,q22,q32) 
 
    Output
    :return: A 4 element array containing the final quaternion (q03,q13,q23,q33) 
 
    """
    # Extract the values from Q0
    w0 = Q0[0]
    x0 = Q0[1]
    y0 = Q0[2]
    z0 = Q0[3]
     
    # Extract the values from Q1
    w1 = Q1[0]
    x1 = Q1[1]
    y1 = Q1[2]
    z1 = Q1[3]
     
    # Computer the product of the two quaternions, term by term
    Q0Q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    Q0Q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    Q0Q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    Q0Q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
     
    # Create a 4 element array containing the final quaternion
    final_quaternion = np.array([Q0Q1_w, Q0Q1_x, Q0Q1_y, Q0Q1_z])
     
    # Return a 4 element array containing the final quaternion (q02,q12,q22,q32) 
    return final_quaternion


def imu_alignment(r_world_sb, r_arb_imu, r_imu_b):
    r_w_imu = quaternion_multiply(r_world_sb, r_arb_imu)
    r_w_b = quaternion_multiply(r_w_imu, r_imu_b)

def weightedAverageQuaternions(Q, w):
    # Average multiple quaternions with specific weights
    # The weight vector w must be of the same length as the number of rows in the
    # quaternion maxtrix Q
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))
    weightSum = 0

    for i in range(0,M):
        q = Q[i,:]
        A = w[i] * numpy.outer(q,q) + A
        weightSum += w[i]

    # scale
    A = (1.0/weightSum) * A

    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = numpy.linalg.eig(A)

    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]

    # return the real part of the largest eigenvector (has only real part)
    return numpy.real(eigenVectors[:,0].A1)


# def get_ekf(H,Q,jacobianH):
#         # Ground Truth
#         x_true = get_status(x_true)

#         # [step1] prediction
#         x_bar = get_status(x_hat)

#         # jacobian matrix
#         jacobianF = get_jacobianF(x_bar)

#         # pre_covariance
#         P_bar = (jacobianF @ P @ jacobianF.T) + Q

#         # observation
#         w = np.random.multivariate_normal([0.0, 0.0], R, 1).T
#         y = (H @ x_true) + w

#         # [step2] update the filter
#         s = (H @ P_bar @ H.T) + R
#         K = (P_bar @ H.T) @ np.linalg.inv(s)

#         # eastimation
#         e = y - (jacobianH @ x_bar)
#         x_hat = x_bar + (K @ e)

#         # post_covariance
#         I = np.identity(x_hat.shape[0])
#         P = (I - K @ H) @ P_bar

#         return x_true, y, x_hat, P


def get_status(x,gyro_angular_vel,dts):
    # get status
    tri = get_trigonometrxic(x)
    Q = np.array([[1, tri[0,1]*tri[1,2], tri[0,0]*tri[1,2]], [0, tri[0,0], -tri[0,1]]])
    x =  x + (np.matmul(Q,gyro_angular_vel)) * dts
    return x


def get_jacobianF(x,gyro_angular_vel,dts):
    # get jacobian matrix of F
    g = gyro_angular_vel
    
    tri = get_trigonometrxic(x)
    jacobianF = np.array([[1.0+(tri[0,0]*tri[1,2]*g[1][0]-tri[0,1]*tri[1,2]*g[2][0])*dts, (tri[0,1]/tri[1,0]/tri[1,0]*g[1][0]+tri[0,0]/tri[1,0]/tri[1,0]*g[2][0])*dts], 
                          [-(tri[0,1]*g[1][0]+tri[0,0]*g[2][0])*dts, 1.0]])
    return jacobianF


def get_trigonometrxic(x):
    # get trigonometrxic of roll&pitch
    return np.array([[np.cos(x[0][0]), np.sin(x[0][0]), np.tan(x[0][0])], [np.cos(x[1][0]), np.sin(x[1][0]), np.tan(x[1][0])]])

def tilting_plus_slamyaw(roll,pitch,slamyaw,r_i2b):

    eul = np.array([-roll + (np.pi/2), -pitch, np.array([slamyaw])])
    # eul = (roll, pitch, slamyaw)
    # print(eul)
    r_w2i = np.array([tf.transformations.quaternion_from_euler(eul[2], eul[1], eul[0],axes='szyx')])
    r_slam = np.array([tf.transformations.quaternion_from_euler(slamyaw,0,0,axes='szyx')])
    q_slam = np.quaternion(r_slam[0][3], r_slam[0][0], r_slam[0][1], r_slam[0][2])
    q_w2i = np.quaternion(r_w2i[0][3], r_w2i[0][0], r_w2i[0][1], r_w2i[0][2])
    q_i2b = np.quaternion(r_i2b[3], r_i2b[0], r_i2b[1], r_i2b[2])
    

    q_b2imuinit = np.quaternion(0,0,0,1)
    r_w2bnoyaw = q_w2i * q_b2imuinit
    # eul_w2bnoywa = np.array([tf.transformations.euler_from_quaternion([r_w2bnoyaw[1], r_w2bnoyaw[2], r_w2bnoyaw[3], r_w2bnoyaw[0]])])
    # eul_w2b = np.array([eul_w2bnoywa, slamyaw])
    # q_w2b = np.array([tf.transformation.euler_from_quaternion(eul_w2b[2], eul_w2b[1], eul_w2b[0], axes="szxy")])
    # q_w2b = q_i2b * q_w2i
    # q_w2b = q_i2b * q_w2i * q_i2b  
    # q_w2b = q_slam * r_w2bnoyaw
    q_w2b = r_w2bnoyaw
    r_w2b = np.array([q_w2b.x, q_w2b.y, q_w2b.z, q_w2b.w])
    # print(eul)
    return r_w2b

def capsd(z_W_R, q_wb, t_wb, q_bi, t_bi, q_is, t_is, r, theta):

    # rot_W_B = R.from_quat(q_wb).as_matrix()
    H_W_B = qt2H(q_wb, t_wb)
    H_B_I = qt2H(q_bi, t_bi)
    H_I_S = qt2H(q_is, t_is)
    # H_W_B = np.array((np.hstack((rot_W_B, t_wb.reshape(-1, 1))), [0, 0, 0, 1]))

    # rot_B_I = R.from_quat(q_bi).as_matrix()
    # H_B_I = np.vstack((np.hstack((rot_B_I, t_bi.reshape(-1, 1))), [0, 0, 0, 1]))
    
    H_W_I = np.matmul(H_W_B, H_B_I)
    H_W_S = np.matmul(H_W_I, H_I_S)

    z_W_R_homo = np.array([0,0,z_W_R,1])

    z_S_R_homo = np.dot(H_W_S, z_W_R_homo)

    z_S_R = z_S_R_homo[2]


    t_sr = f(r, theta, z_S_R)

    q_sr = np.array([0,0,0,1])
    H_S_R = qt2H(q_sr, t_sr)
    # print(H_S_R)    
    H_W_R = np.matmul(H_W_S, H_S_R)
    p_wr = np.array([H_W_R[0][3], H_W_R[1][3], z_W_R])
    # print(p_wr)
    return p_wr



def f(r, theta_deg, d):
    r_ = math.sqrt(r**2 - d**2)
    theta = np.radians(-theta_deg)
    # x = r_ * math.cos(theta)
    # y = r_ * math.sin(theta)

    x = r_ *math.cos(theta)
    y = r_ *math.sin(theta)    

    p_sr = [x,y,d]
    # print(p_sr)
    return p_sr

def pixel_to_sonar_beam(width, height, fov_deg, max_range, pixel_x, pixel_y):

    # relative_x = pixel_x - width / 2.0
    # relative_y = height - pixel_y  

    v1 = np.array([pixel_x-width/2, pixel_y-height])
    v2 = np.array([0, -height])
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    cos_angle = dot_product / (norm_v1 * norm_v2)
    angle_radians = np.arccos(np.clip(cos_angle, -1.0, 1.0))
    angle_degrees = np.degrees(angle_radians)
    # theta_per_pixel = fov_deg / width
    # theta = theta_per_pixel * relative_x 
    # theta's unit: degree
    if pixel_x < width/2:
        angle_degrees = angle_degrees
    else:
        angle_degrees = -angle_degrees

    pixel_distance = math.sqrt((width / 2 - pixel_x)**2 + (height - pixel_y)**2)
    r = max_range / height * pixel_distance
    # print(r)
    # r_per_pixel = max_range / height
    # r = r_per_pixel * relative_y  
    # print(r, angle_degrees)
    return r, angle_degrees

def sonar_pixel_to_distance_angle_adjusted(img_width, img_height, fov_deg, max_range,x, y):
    """
    Calculate the distance and angle of a point in a sonar image relative to the sonar device,
    considering the effective width from 80 to 560 pixels instead of from 0 to 640 pixels.

    Parameters:
    x (int): The x coordinate of the point in the image, within the effective range [80, 560].
    y (int): The y coordinate of the point in the image.
    img_width (int): The effective width of the sonar image in pixels, which is 480 pixels in this scenario.
    img_height (int): The height of the sonar image in pixels.
    fov_deg (float): The field of view of the sonar in degrees.
    max_range (float): The maximum range of the sonar in the same units as the desired distance.

    Returns:
    tuple: A tuple containing the distance and angle (in degrees) of the point from the sonar device.
    """
    # Adjust x coordinate to start from the effective beginning (80 pixels)
    adjusted_x = x - 80

    # Calculate the effective width of the image
    effective_width = 560 - 80

    # Convert the adjusted x coordinate to an angle relative to the center line of the sonar beam
    angle_per_pixel = fov_deg / effective_width
    angle = (adjusted_x - effective_width / 2.0) * angle_per_pixel

    # Convert the y coordinate to a distance
    distance = (y / img_height) * max_range

    return distance, angle


# def f(a, b, c, r, theta_deg, d):

#     theta = np.radians(theta_deg)
    
    
#     def equations(vars):
#         x, y, z = vars
#         eq1 = (x - a)**2 + (y - b)**2 + (z - c)**2 - r**2
#         eq2 = z - d
#         eq3 = (z - c) - np.tan(theta) * (x - a)
#         return [eq1, eq2, eq3]
    
    
#     initial_guess = [a, b, d]
    
#     # 使用fsolve求解方程
#     solution = fsolve(equations, initial_guess)
#     print(solution)
#     # 检查解是否满足x > 0的条件
#     if solution[0] > 0:
#         return solution
#     else:
#         return "No solution found with x > 0"



# def f(x_W_S, y_W_S, z_W_S, R, theta, z_depth):

  
#     tan_theta = np.tan(theta)

    
#     A = 1 + tan_theta**2
#     B = -2 * (x_W_S + y_W_S * tan_theta)
#     C = x_W_S**2 + y_W_S**2 + (z_depth - z_W_S)**2 - R**2

    
#     coefficients = [A, B, C]
#     roots = np.roots(coefficients)
#     X_positive = roots[roots > 0]

#     if X_positive.size > 0:
#         X = X_positive[0] 
#         Y = X * tan_theta
#         sol = [X,Y, z_depth]
#         return sol
#     else:
#         print("No solution found with X > 0.") 






# def tilting_plus_slamyaw(roll,pitch,slamyaw,r_i2b):
#     # print(roll)
#     # np.quaternion(w,x,y,z)
#     q_imuref =  np.quaternion(0.7071,0,0, 0.7071)
#     q_cali = tf.transformations.quaternion_conjugate([q_imuref.w, q_imuref.x, q_imuref.y, q_imuref.z])
#     q_cali = np.quaternion(q_cali[0], q_cali[1], q_cali[2], q_cali[3])
#     # print(q_cali)
#     # print(pitch)
#     eul = np.array([-(roll-1.5), pitch, np.array(slamyaw)])
#     # eul = (roll, pitch, slamyaw)
#     # print(eul)
#     r_w2i = np.array([tf.transformations.quaternion_from_euler(eul[0], eul[1], eul[2],axes='sxyz')])
#     r_slam = np.array([tf.transformations.quaternion_from_euler(slamyaw,0,0,axes='szyx')])
#     q_slam = np.quaternion(r_slam[0][3], r_slam[0][0], r_slam[0][1], r_slam[0][2])
#     q_w2i = np.quaternion(r_w2i[0][3], r_w2i[0][0], r_w2i[0][1], r_w2i[0][2])


#     q_b2imuinit = np.quaternion(0,0,0,1)
#     r_w2bnoyaw = q_w2i * q_b2imuinit
#     # eul_w2bnoywa = np.array([tf.transformations.euler_from_quaternion([r_w2bnoyaw[1], r_w2bnoyaw[2], r_w2bnoyaw[3], r_w2bnoyaw[0]])])
#     # eul_w2b = np.array([eul_w2bnoywa, slamyaw])
#     # q_w2b = np.array([tf.transformation.euler_from_quaternion(eul_w2b[2], eul_w2b[1], eul_w2b[0], axes="szxy")])
#     # q_w2b = q_i2b * q_w2i
#     # q_w2b = q_i2b * q_w2i * q_i2b  
#     # q_w2b = q_slam * r_w2bnoyaw
#     q_w2b = q_w2i
#     r_w2b = np.array([q_w2b.x, q_w2b.y, q_w2b.z, q_w2b.w])
#     # print(eul)
#     return r_w2b

def tilting_plus_slamyaw(roll,pitch,slamyaw,rolloffset):
    # print(slamyaw)
    # np.quaternion(w,x,y,z)
    # q_imuref =  np.quaternion(0.7071,0,0, 0.7071)
    # q_cali = tf.transformations.quaternion_conjugate([q_imuref.w, q_imuref.x, q_imuref.y, q_imuref.z])
    # q_cali = np.quaternion(q_cali[0], q_cali[1], q_cali[2], q_cali[3])
    # print(q_cali)

    eul = (-(roll-rolloffset), pitch, slamyaw)
    # eul = (roll, pitch, slamyaw)
    # print(eul)
    r_w2i = np.array([tf.transformations.quaternion_from_euler(eul[2], eul[1], eul[0],axes='szyx')])
    # r_slam = np.array([tf.transformations.quaternion_from_euler(slamyaw,0,0,axes='szyx')])
    # q_slam = np.quaternion(r_slam[0][3], r_slam[0][0], r_slam[0][1], r_slam[0][2])
    q_w2i = np.quaternion(r_w2i[0][3], r_w2i[0][0], r_w2i[0][1], r_w2i[0][2])


    # q_b2imuinit = np.quaternion(0,0,0,1)
    # r_w2bnoyaw = q_w2i * q_b2imuinit
    # eul_w2bnoywa = np.array([tf.transformations.euler_from_quaternion([r_w2bnoyaw[1], r_w2bnoyaw[2], r_w2bnoyaw[3], r_w2bnoyaw[0]])])
    # eul_w2b = np.array([eul_w2bnoywa, slamyaw])
    # q_w2b = np.array([tf.transformation.euler_from_quaternion(eul_w2b[2], eul_w2b[1], eul_w2b[0], axes="szxy")])
    # q_w2b = q_i2b * q_w2i
    # q_w2b = q_i2b * q_w2i * q_i2b  
    # q_w2b = q_slam * r_w2bnoyaw
    q_w2b = q_w2i
    r_w2b = np.array([q_w2b.x, q_w2b.y, q_w2b.z, q_w2b.w])
    # print(eul)
    return r_w2b


