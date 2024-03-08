#!/usr/bin/env python3
import rospy
import subprocess

def run_joint_manipulator(end_state):
    length = float(input("Enter length:"))
    rospy.loginfo(f"Running joint manipulator with end_state: {end_state} and length: {length}")
    try:
        cmd = ["rosrun", "monkeybot_autonomous", "joint_manipulator_ba.py", "--end_state", end_state, "--length", str(length)]
        subprocess.Popen(cmd)
    except Exception as e:
        rospy.logerr(f"Failed to launch joint_manipulator: {e}")


if __name__ == '__main__':
    run_joint_manipulator("home")