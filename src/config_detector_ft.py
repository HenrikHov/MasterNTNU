"""
This is meant to be a sudo code that could work in theory with integrated force-torque sensors.
Force-torque sensors have not been integrated on the monkeybot. But this should work as a 
baseline, if such an aproach for determening the configuration would be taken into consideration.
This code follows the same structure as prox_ab_ba_config.py file.
It also uses the same name convention as the monkeybot.
Note that is necesarely doesnt need to be a force-torque sensor, it could be as simple as buttons
integrated into the finger of the monkeybot.
"""

#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Bool, Float64

# There could be error readings from the force-torque sensor when it isnt touching anything, or some
# movement when the monkeybot is moving. This supports as a threshold to avoid this potentiol error.
# Change value as necessary.
force_threshold = 10.0

def set_config(sensor_list_a, sensor_list_b):
    total_a = sum(sensor_list_a.values())
    total_b = sum(sensor_list_b.values())

    if total_a > force_threshold and total_b > force_threshold:
        return "both"
    elif total_a > force_threshold:
        return "ab"
    elif total_b > force_threshold:
        return "ba"
    else:
        return rospy.logerr(f"Failed to determine configuration. Investigate sensors and 
                            force threshold: {e}")
    
# Each finger is given 1 force-torque sensor. 
def check_force():
    sensor_readings_a = {'a00': None, 'a01': None}
    sensor_readings_b = {'b00': None, 'b01': None}

    def sensor_callback_a(data, sensor_id):
        nonlocal sensor_readings_a
        sensor_readings_a[sensor_id] = data.force

    def sensor_callback_b(data, sensor_id):
        nonlocal sensor_readings_b
        sensor_readings_b[sensor_id] = data.force

    # Subscribe to a sensor topics
    for sensor_id_a in sensor_readings_a:
        topic = f"/force_torque_sensor_{sensor_id_a}/force"
        rospy.Subscriber(topic, Float64, sensor_callback_a, sensor_id_a)

    # Subscribe to b sensor topics
    for sensor_id_b in sensor_readings_b:
        topic = f"/force_torque_sensor_{sensor_id_b}/force"
        rospy.Subscriber(topic, Float64, sensor_callback_b, sensor_id_b)
    
    # Wait for all sensor values to be received
    while not all(value is not None for value in sensor_readings_a.values()) or \
          not all(value is not None for value in sensor_readings_b.values()):
        rospy.sleep(0.1)  # Adjust this as needed

    return set_config(sensor_readings_a, sensor_readings_b)

if __name__ == "__main__":
    rospy.init_node('config_detector_ft', anonymous=True)
    configuration = check_force()
    print(f"Configuration: {configuration}")
