#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range

def check_proximity():
    sensor_readings = {'b00': None, 'b01': None, 'b02': None, 'b03': None}
    safe_to_dock = False # Variable to hold the docking status

    def sensor_callback(data, sensor_id):
        nonlocal safe_to_dock # This allows us to modify the outer variable
        sensor_readings[sensor_id] = data.range
        if all(val is not None for val in sensor_readings.values()) and all(val < 0.3 for val in sensor_readings.values()):
            safe_to_dock = True

    # Subscribe to sensor topics
    for sensor_id in sensor_readings:
        topic = f"/proximity_sensor_{sensor_id}/sonar"
        rospy.Subscriber(topic, Range, sensor_callback, sensor_id)

    
    # Wait for sensor values to update
    rospy.sleep(2)  # Adjust time as needed
    
    # Check if all readings are below 0.3 meters
    return safe_to_dock

# The following allows this file to be imported as a module without executing any code
if __name__ == "__main__":
    rospy.init_node('proximity_detection', anonymous=True)
    result = check_proximity()
    print(f"Safe to dock: {result}")

