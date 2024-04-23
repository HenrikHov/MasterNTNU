"""
This has the same idea as ft_ab_ba_config.py, but with a simple button integration instead of a 
force-torque sensor.
"""

#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Bool

# For safety reason both buttons on a gripper has to be pressed for that gripper to be viewed as 
# being attached to a pipe.
def set_config(sensor_list_a, sensor_list_b):
    total_a = sum(sensor_list_a.values())
    total_b = sum(sensor_list_b.values())

    if total_a == 2 and total_b == 2:
        return "both"
    elif total_a > total_b:
        return "ab"
    elif total_a < total_b:
        return "ba"
    else:
        return rospy.logerr(f"Failed to determine configuration. Investigate buttons and code: {e}")

# Each finger is given 1 button. 
def check_button():
    sensor_readings_a = {'a00': None, 'a01': None}
    sensor_readings_b = {'b00': None, 'b01': None}

    def sensor_callback_a(data, sensor_id):
        nonlocal sensor_readings_a
        sensor_readings_a[sensor_id] = data.data

    def sensor_callback_b(data, sensor_id):
        nonlocal sensor_readings_b
        sensor_readings_b[sensor_id] = data.data

    for sensor_id_a in sensor_readings_a:
        topic = f"/button_{sensor_id_a}/button"
        rospy.Subscriber(topic, Bool, sensor_callback_a, sensor_id_a)

    for sensor_id_b in sensor_readings_b:
        topic = f"/button_{sensor_id_b}/button"
        rospy.Subscriber(topic, Bool, sensor_callback_b, sensor_id_b)
    
    while not all(value is not None for value in sensor_readings_a.values()) or \
          not all(value is not None for value in sensor_readings_b.values()):
        rospy.sleep(0.1)

    return set_config(sensor_readings_a, sensor_readings_b)

if __name__ == "__main__":
    rospy.init_node('config_detector_button', anonymous=True)
    configuration = check_button()
    print(f"Configuration: {configuration}")
