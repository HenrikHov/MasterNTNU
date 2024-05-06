import math
import numpy as np
import rospy

def walking(input_length):
    step_distance = 1.2
    total_length = input_length
    total_steps = math.ceil(total_length/step_distance)
    distances = np.zeros(total_steps)
    whole_steps = total_length // step_distance
    remaining_length = total_length % step_distance
    for i in range(whole_steps):
        distances[i] = step_distance
    if remaining_length < 0.5:
        diff = 0.5 - remaining_length
        distances[total_steps-1] = step_distance - diff
        remaining_length = diff + remaining_length
    distances[total_steps] = remaining_length
    return distances

def main(input_length):
    input_length = float(input("Enter length:"))
    if abs(input_length) > 1.2:
        return walking(input_length)
    elif abs (input_length) < 0.5:
        rospy.loginfo("Length input too small, try again with a larger or smaller value than 0.5 or -0.5")
    else:
        return input_length



