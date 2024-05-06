import math
import numpy as np

def walking(input_length):
    multiple_sensors = str(input("Attache multiple sensors?[y/n]"))
    if multiple_sensors == "y":
        n_sensors = int(input("Number of sensors:"))
        step_distance = float(input("Distance between sensors"))
        total_length = n_sensors * step_distance
        total_steps = n_sensors
    else:
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


