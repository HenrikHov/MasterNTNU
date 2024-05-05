import math
import numpy as np
from autonomous_launcher import launch

def walking(input_length):
    walking_distance = 1.2
    total_length = input_length
    max_length = walking_distance
    total_steps = math.ceil(total_length/max_length)
    distances = np.zeros(total_steps)
    whole_steps = total_length // max_length
    remaining_length = total_length % max_length
    for i in range(whole_steps):
        distances[i] = walking_distance
    if remaining_length < 0.5:
        diff = 0.5 - remaining_length
        distances[total_steps-1] = diff
    distances[total_steps] = remaining_length
    return distances

