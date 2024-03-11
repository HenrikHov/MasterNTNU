#!/usr/bin/env python3
import math
import numpy as np
import modern_robotics as mr
import random
import rospy
from std_msgs.msg import Float64MultiArray


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
    temp_angle[2] = abs(temp_angle[2])
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
    total_angle[2] = -abs(total_angle[2])

    return total_angle


def inverse_kinematics(configuration, end_state, init_length):
    
    # Define the robot's home configuration (M)
    M = np.array([[1, 0, 0, 0],
                [0, -1, 0, 0],
                [0, 0, -1, 1.400],
                [0, 0, 0, 1]])

    # Rotation axis for each joint
    w0a = np.array([1, 0, 0])
    w3a = np.array([-1, 0, 0])
    w3b = np.array([1, 0, 0])
    w0b = np.array([1, 0, 0])

    # Distance from joint to the end-effector
    q0a = np.array([-0.1495, 0, 1.400])
    q3a = np.array([0, 0, 0.8665])
    q3b = np.array([0, 0, 0.5335])
    q0b = np.array([0.1495, 0, 0])


    w = np.array([w0b, w3b, w3a, w0a])
    q = np.array([q0b, q3b, q3a, q0a])

    # Calculates the cross product of w and q
    v = np.zeros((4, 3))

    for i in range(0, len(w)):
        v[i] = np.cross(-w[i], q[i])

    # Creates the joint screw table expressed in the end-effector frame
    Blist = np.hstack((w, v)).T

    if init_length > 0:
        length_sgn = 1
        length = init_length

    elif init_length < 0:
        length_sgn = -1
        length= (-1) * init_length

    if end_state == "home":
        height  = 0.2
        thetalist0 = np.array([0.2, 1.2, -0.9, 0.7])

    elif end_state == "dock":
        height = 0
        thetalist0 = np.array([0.4, 1.1, -1.1, 0.4])

    T = np.array([[1, 0, 0, 0],
                [0, 1, 0, length],
                [0, 0, 1, height],
                [0, 0, 0, 1]])


    # Set the error tolerances
    eomg = 1e-50
    ev = 1e-50
    attempt = 0

    rospy.loginfo("Calculating inverse kinematics...")
    while True:
        if attempt == 1000:
            rospy.loginfo("Was not able to find a solution after 1000 attempts. Aboarts operation")
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
                sum = final_adjustment[0] + final_adjustment[1] + (-1)*final_adjustment[2] + final_adjustment[3]
                
                if length_sgn == -1:
                    final_adjustment = length_sgn * final_adjustment
                break

            else:
                # print("Inverse kinematics failed to converge. Attempt:", attempt)
                hundred_max = math.pi*100/180
                thetalist0 = np.array([random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max),
                                    random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max)])
            
        else:
            # print("Inverse kinematics failed to converge. Attempt:", attempt)
            hundred_max = math.pi*100/180
            thetalist0 = np.array([random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max),
                                    random.uniform(-hundred_max, hundred_max), random.uniform(-hundred_max, hundred_max)])

    return final_adjustment *math.pi/180




