#!/usr/bin/env python3
import argparse
import rospy
import subprocess

def run_joint_manipulator(end_state, length):
    rospy.loginfo(f"Running joint manipulator with end_state: {end_state} and length: {length}")
    try:
        cmd = ["rosrun", "monkeybot_autonomous", "joint_manipulator_ab.py", "--end_state", end_state, "--length", str(length)]
        subprocess.Popen(cmd)
    except Exception as e:
        rospy.logerr(f"Failed to launch joint_manipulator: {e}")

if __name__ == '__main__':
    rospy.init_node("config_launcher")
    parser = argparse.ArgumentParser(description="Launcher for Joint Manipulator")
    parser.add_argument("--length", type=float, default=1.0, help="Length value")
    args = parser.parse_args()

    run_joint_manipulator("home", args.length)