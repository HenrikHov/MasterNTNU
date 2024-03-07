#!/usr/bin/env python3
import sys
import rospy
import subprocess
import threading
from sensor_msgs.msg import Range
import moveit_commander
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander


# Global variables to track sensor readings and RViz launch status
sensor_readings = {'b00': None, 'b01': None, 'b02': None, 'b03': None}
rviz_launched = False
launch_lock = threading.Lock()

# Callback function for sensor data
def sensor_callback(data, sensor_id):
    global rviz_launched
    with launch_lock:  # Ensures thread-safe operations
        sensor_readings[sensor_id] = data.range
        # Check if all sensors have reported and their values are less than 0.2 meters
        if not rviz_launched and all(val is not None for val in sensor_readings.values()) and all(val < 0.4 for val in sensor_readings.values()):
            run_rviz_launch()
            rviz_launched = True  # Prevent further launches

# Function to run rviz.launch
def run_rviz_launch():
    rospy.loginfo("All sensors reported less than 0.4 meters, launching rviz.launch.")
    try:
        #  subprocess.call(["roslaunch", "monkeybot_bringup", "rviz.launch"])
        subprocess.call(["rosrun", "monkeybot_autonomous", "Inverse_Kinematics_3.py"])
        rospy.sleep(5)  # Wait for RViz to fully launch
        
    except Exception as e:
        rospy.logerr("Failed to launch rviz.launch: %s" % e)

# Main listener function
def listener():
    rospy.init_node('proximity_sensor_listener', anonymous=True)

    rospy.Subscriber("/proximity_sensor_b00/sonar", Range, sensor_callback, callback_args="b00")
    rospy.Subscriber("/proximity_sensor_b01/sonar", Range, sensor_callback, callback_args="b01")
    rospy.Subscriber("/proximity_sensor_b02/sonar", Range, sensor_callback, callback_args="b02")
    rospy.Subscriber("/proximity_sensor_b03/sonar", Range, sensor_callback, callback_args="b03")

    rospy.spin()

if __name__ == '__main__':
    listener()
