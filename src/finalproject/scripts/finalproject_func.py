#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm, logm
from lab5_header import *
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
	# =================== Your code starts here ====================#
    # Fill in the correct values for a1~6 and q1~6, as well as the M matrix
    M = np.eye(4)
    S = np.zeros((6,6))

    v1, v2, v3 = getCross(0, 0, 1, -.150, .150, .010)
    S1 = Get_S(0, 0, 1, v1, v2, v3)

    v1, v2, v3 = getCross(0, 1, 0, -.150, .270, .162)
    S2 = Get_S(0, 1, 0, v1, v2, v3)

    v1, v2, v3 = getCross(0, 1, 0, .094, .270, .162)
    S3 = Get_S(0, 1, 0, v1, v2, v3)

    v1, v2, v3 = getCross(0, 1, 0, .307, .177, .162)
    S4 = Get_S(0, 1, 0, v1, v2, v3)

    v1, v2, v3 = getCross(1, 0, 0, .307, .260, .162)
    S5 = Get_S(1, 0, 0, v1, v2, v3)

    v1, v2, v3 = getCross(0, 1, 0, .390, .260, .162)
    S6 = Get_S(0, 1, 0, v1, v2, v3)

    S = np.array([S1, S2, S3, S4, S5, S6])
    M = np.array([[0, -1, 0, .390], [0, 0, -1, .401], [1, 0, 0, .2155], [0, 0, 0, 1]])


    # ==============================================================#
    return M, S

"""
Function that returns [Si]
"""
def Get_S(w1, w2, w3, v1, v2, v3):
    return np.array([[0, -w3, w2, v1],[w3, 0, -w1, v2], [-w2, w1, 0, v3],[0, 0, 0, 0]])

"""
Function that gets cross product of vectors -w and q
"""
def getCross(w1, w2, w3, q1, q2, q3):
    w = [-w1, -w2, -w3]
    q = [q1, q2, q3]
    v = np.cross(w, q)
    v1 = v[0]
    v2 = v[1]
    v3 = v[2]
    return v1, v2, v3

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

    # Initialize the return_value 
    return_value = [None, None, None, None, None, None]

    print("Foward kinematics calculated:\n")

    # =================== Your code starts here ====================#
    thetas = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
    T = np.eye(4)

    M, S = Get_MS()

    # calculating first matrix (S1* theta1)
    exponent = np.array(thetas[0] * S[0])
    prev_mat = expm(exponent)

    # calculate prev_mat to multiply M matrix with 
    for i in range (1,len(thetas)):
            exponent = np.array(thetas[i] * S[i])
            prev_mat = np.matmul(prev_mat, expm(exponent))

    # multiply M matrix and calculated value
    T = np.matmul(prev_mat, M)
    print(T)

    # ==============================================================#

    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*PI)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#

    t5 = -PI/2
    # t2 = 0
    # t3 = 0
    # # t4 = 0

    xgrip = xWgrip + .150 
    ygrip = yWgrip - .150 
    zgrip = zWgrip - .010

    # *** variables used in following calculations *** #
    L1 = .152
    L2 = .120
    L3 = .244
    L4 = .093
    L5 = .213
    L6 = .083
    L7 = .083
    L8 = .082
    L9 = .0535
    L10 = .059
    dist_to_3end = .027
    yaw_radians = math.radians(yaw_WgripDegree)
    # ************************************************ #

    xcen = xgrip - L9 * math.cos(yaw_radians)
    ycen = ygrip - L9 * math.sin(yaw_radians)
    zcen = zgrip

    hyp = math.sqrt(xcen**2 + ycen**2)
    t1 = math.atan2(ycen, xcen) - math.asin((dist_to_3end + L7)/hyp)
    t6 = t1 + PI/2 - yaw_radians

    xproj1 = (dist_to_3end + L6) * math.sin(t1)
    yproj1 = (dist_to_3end + L6) * math.cos(t1)
    xproj2 = L7 * math.cos(t1)
    yproj2 = L7 * math.sin(t1)

    x3end = xcen + xproj1 - xproj2
    y3end = ycen - yproj1 - yproj2
    z3end = zcen + L8 + L10

    # theta 3
    a = L3 
    b = L5
    c = math.sqrt((x3end)**2 + (y3end)**2 + (L1 - z3end)**2)
    x = (c**2 - a**2 - b**2)
    y = (-1 * x) / (2 * a * b)
    t3 = PI - math.acos(y)

    # theta 2
    a = L3 
    b = math.sqrt((x3end)**2 + (y3end)**2 + (L1 - z3end)**2)
    c = L5
    x = (c**2 - a**2 - b**2)
    y = (-1 * x) / (2 * a * b)
    t2a = math.acos(y)
    t2b = math.acos((math.sqrt(x3end**2 + y3end**2)) / b)
    t2 = -1 * (t2a + t2b)

    # theta 4
    t4 = -1 * (2 * PI - (-1 * t2 + (PI - t3) + PI/2 + PI/2))

    return lab_fk(float(t1), float(t2), float(t3), float(t4), float(t5), float(t6))

    # ==============================================================#
    pass