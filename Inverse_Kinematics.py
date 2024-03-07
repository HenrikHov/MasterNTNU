#!/usr/bin/env python3
import math
import numpy as np
import modern_robotics as mr
import random

def joint_tolerance_tangent(theta):
    check_list = np.zeros((1, len(theta)))

    for i in range(len(check_list)):
        if abs(theta[i]) > 100:
            check_list[i] = 1
        else:
            check_list[i] = 0
    return check_list

def joint_tolerance_axial(theta):
    check_list = np.zeros((1, len(theta)))
    
    for i in range(len(check_list)):
        if abs(theta[i]) > 360:
            check_list[i] = 1
        else:
            check_list[i] = 0
    return check_list


# Define the robot's home configuration (M)
M = np.array([[1, 0, 0, 0],
              [0, -1, 0, 0],
              [0, 0, -1, 1.400],
              [0, 0, 0, 1]])

# Rotation axis for each joint
w0a = np.array([-1, 0, 0])
w1a = np.array([0, -1, 0])
w2a = np.array([0, 0, -1])
w3a = np.array([1, 0, 0])
w3b = np.array([-1, 0, 0])
w2b = np.array([0, 0, -1])
w1b = np.array([0, -1, 0])
w0b = np.array([-1, 0, 0])

# Distance from joint to the end-effector
q0a = np.array([-0.1495, 0, 1.400])
q1a = np.array([0, 0, 1.200])
q2a = np.array([0, 0, 0.9365])
q3a = np.array([0, 0, 0.8665])
q3b = np.array([0, 0, 0.5335])
q2b = np.array([0, 0, 0.4635])
q1b = np.array([0, 0, 0.200])
q0b = np.array([0.1495, 0, 0])


w = np.array([w0a, w1a, w2a, w3a, w3b, w2b, w1b, w0b])
q = np.array([q0a, q1a, q2a, q3a, q3b, q2b, q1b, q0b])

# Calculates the cross product of w and q
v = np.zeros((8, 3))

for i in range(0, len(w)):
    v[i] = np.cross(-w[i], q[i])

# Creates the joint screw table expressed in the end-effector frame
Blist = np.hstack((w, v)).T

T = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0.7],
              [0, 0, 1, 0.2],
              [0, 0, 0, 1]])

# Defines the initial guess for the joint angles
thetalist0 = np.array([0, 0, 0, -1.22, 0.87, 0, 0, 0.87])

# Set the error tolerances
eomg = 1e-10
ev = 1e-10

while True:
    [thetalist, success] = mr.IKinBody(Blist, M, T, thetalist0, eomg, ev)

    # Check if the solution is successful and output the joint angles
    if success:
        thetalist_angles = thetalist*180/math.pi

        for i in range(len(thetalist_angles)):
            if thetalist_angles[i] > 360:
                while thetalist_angles[i] > 360:
                    thetalist_angles[i] -= 360
                    if thetalist_angles[i] < 360:
                        break
            if thetalist_angles[i] < -360:
                while thetalist_angles[i] < -360:
                    thetalist_angles[i] += 360
                    if thetalist_angles[i] > -360:
                        break
    
        
        joint_tangent = np.array([thetalist_angles[0], thetalist_angles[1], thetalist_angles[3],
                                  thetalist_angles[4], thetalist_angles[6], thetalist_angles[7]])
        joint_axial = np.array([thetalist_angles[2], thetalist_angles[5]])

        joint_tolerance = np.hstack((joint_tolerance_tangent(joint_tangent), joint_tolerance_axial(joint_axial)))

        if np.all(joint_tolerance == 0):
            thetalist_angles[1], thetalist_angles[2], thetalist_angles[5], thetalist_angles[6] = 0, 0, 0, 0
            print(thetalist_angles)
            break
        else:
            hundred_max = math.pi*100/180
            three_six_max = 2*math.pi
            thetalist0 = np.array([random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max),
                           random.uniform(-three_six_max, three_six_max), random.uniform(-hundred_max, hundred_max),
                           random.uniform(-hundred_max, hundred_max), random.uniform(-three_six_max, three_six_max),
                           random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max)])
        
    else:
        print("Inverse kinematics failed to converge")
        hundred_max = math.pi*100/180
        three_six_max = 2*math.pi
        thetalist0 = np.array([random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max),
                           random.uniform(-three_six_max, three_six_max), random.uniform(-hundred_max, hundred_max),
                           random.uniform(-hundred_max, hundred_max), random.uniform(-three_six_max, three_six_max),
                           random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max)])


