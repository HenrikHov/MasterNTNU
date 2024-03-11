#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range

def set_config(sensor_list_a, sensor_list_b):
    total_a = sum(sensor_list_a.values())
    total_b = sum(sensor_list_b.values())

    if total_a < total_b:
        return "ab"
    else:
        return "ba"

def check_proximity():
    sensor_readings_a = {'a00': None, 'a01': None, 'a02': None, 'a03': None}
    sensor_readings_b = {'b00': None, 'b01': None, 'b02': None, 'b03': None}

    def sensor_callback_a(data, sensor_id):
        nonlocal sensor_readings_a
        sensor_readings_a[sensor_id] = data.range

    def sensor_callback_b(data, sensor_id):
        nonlocal sensor_readings_b
        sensor_readings_b[sensor_id] = data.range

    # Subscribe to a sensor topics
    for sensor_id_a in sensor_readings_a:
        topic = f"/proximity_sensor_{sensor_id_a}/sonar"
        rospy.Subscriber(topic, Range, sensor_callback_a, sensor_id_a)

    # Subscribe to b sensor topics
    for sensor_id_b in sensor_readings_b:
        topic = f"/proximity_sensor_{sensor_id_b}/sonar"
        rospy.Subscriber(topic, Range, sensor_callback_b, sensor_id_b)

    # Wait for all sensor values to be received
    while not all(value is not None for value in sensor_readings_a.values()) or \
          not all(value is not None for value in sensor_readings_b.values()):
        rospy.sleep(0.1)  # Adjust this as needed

    return set_config(sensor_readings_a, sensor_readings_b)

if __name__ == "__main__":
    rospy.init_node('proximity_check_test', anonymous=True)
    configuration = check_proximity()
    print(f"Configuration: {configuration}")
