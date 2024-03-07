#!/usr/bin/env python
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
    rospy.loginfo("All sensors reported less than 0.3 meters, launching rviz.launch.")
    try:
        subprocess.call(["roslaunch", "monkeybot_bringup", "rviz.launch"])
        rospy.sleep(5)  # Wait for RViz to fully launch
        set_joint_positions()
    except Exception as e:
        rospy.logerr("Failed to launch rviz.launch: %s" % e)

# Function to set robot arm to a specific joint configuration
def set_joint_positions():
    moveit_commander.roscpp_initialize(sys.argv)
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group_name = "arm"  # Change to your MoveIt group name
    move_group = MoveGroupCommander(group_name)

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.22689280276  # ab_joint0a
    joint_goal[1] = 0  # ab_joint1a
    joint_goal[2] = 0  # ab_joint2a
    joint_goal[3] = -0.99483767364  # ab_joint3a
    joint_goal[4] = 1.2042771839  # ab_joint3b
    joint_goal[5] = 0  # ab_joint2b
    joint_goal[6] = 0  # ab_joint1b
    joint_goal[7] = 0.73303828584  # ab_joint0b

    move_group.go(joint_goal, wait=True)
    move_group.stop()  # Ensure there's no residual movement

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
