#!/usr/bin/env python3
import math
import numpy as np
import modern_robotics as mr
import random

def joint_tolerance_check(theta):
    check_list = np.zeros(len(theta))

    for i in range(len(theta)):
        if abs(theta[i]) > 97:
            check_list[i] = 1
        else:
            check_list[i] = 0
    return check_list

# This function is only aplicable on an axial cylinder at the moment
def angle_precision(total_angle):
    if len(total_angle) != 4:
        raise ValueError("The input array must contain exactly 4 values.")

    # Use the absolute value of the second element for sum calculation
    temp_angle = total_angle.copy()
    temp_angle[1] = abs(temp_angle[1])
    current_sum = sum(temp_angle)

    # Determine whether the target sum is 180 or -180
    target_sum = 180 if abs(180 - current_sum) < abs(-180 - current_sum) else -180

    # Find the index of the value closest to 0, excluding the second value
    closest_to_zero_index = min(range(len(total_angle)), key=lambda i: abs(total_angle[i]) if i != 1 else float('inf'))

    # Calculate the adjustment needed to reach the target sum
    adjustment = target_sum - current_sum

    # Adjust the closest-to-zero value
    total_angle[closest_to_zero_index] += adjustment

    # Ensure the second element remains negative
    total_angle[1] = -abs(total_angle[1])

    return total_angle




# Define the robot's home configuration (M)
M = np.array([[1, 0, 0, 0],
              [0, -1, 0, 0],
              [0, 0, -1, 1.400],
              [0, 0, 0, 1]])

# Rotation axis for each joint
w0a = np.array([-1, 0, 0])
w3a = np.array([1, 0, 0])
w3b = np.array([-1, 0, 0])
w0b = np.array([-1, 0, 0])

# Distance from joint to the end-effector
q0a = np.array([-0.1495, 0, 1.400])
q3a = np.array([0, 0, 0.8665])
q3b = np.array([0, 0, 0.5335])
q0b = np.array([0.1495, 0, 0])


w = np.array([w0a, w3a, w3b, w0b])
q = np.array([q0a, q3a, q3b, q0b])

# Calculates the cross product of w and q
v = np.zeros((4, 3))

for i in range(0, len(w)):
    v[i] = np.cross(-w[i], q[i])

# Creates the joint screw table expressed in the end-effector frame
Blist = np.hstack((w, v)).T

length = float(input("Lengde: "))
T = np.array([[1, 0, 0, 0],
              [0, 1, 0, length],
              [0, 0, 1, 0.2],
              [0, 0, 0, 1]])

# Defines the initial guess for the joint angles
thetalist0 = np.array([0.2, -0.9, 1.2, 0.7])

# Set the error tolerances
eomg = 1e-50
ev = 1e-50
attempt = 0

while True:
    if attempt == 1000:
        print("Was not able to find a solution after 1000 attempts. Aboarts operation")
        break

    attempt += 1
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
    
        joint_tolerance = joint_tolerance_check(thetalist_angles)

        if np.all(joint_tolerance == 0):
            final_adjustment = angle_precision(thetalist_angles)
            sum = final_adjustment[0] + (-1)*final_adjustment[1] + final_adjustment[2] + final_adjustment[3]
            print("Found a solution after:", attempt, "attempts.")
            print("Length:", length)
            print("Angle:", sum)
            print("Joint states:", final_adjustment)
            break

        else:
            print("Inverse kinematics failed to converge. Attempt:", attempt)
            hundred_max = math.pi*100/180
            thetalist0 = np.array([random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max),
                                   random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max)])
        
    else:
        print("Inverse kinematics failed to converge. Attempt:", attempt)
        hundred_max = math.pi*100/180
        thetalist0 = np.array([random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max),
                                   random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max)])

