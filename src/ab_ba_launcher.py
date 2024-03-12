#!/usr/bin/env python3
import rospy
import subprocess
from prox_ab_ba_config import check_proximity

def configuration():
    rospy.loginfo("Starting configuration determination.")
    config = check_proximity()
    rospy.loginfo(f"Configuration determined: {config}")
    if config == "both":
        config = str(input("Enter config [ab/ba]:"))
        
    length = float(input("Enter length:"))
    rospy.loginfo(f"Launching with config: {config} and length: {length}")

    # Launch corresponding Python script based on configuration
    if config in ["ab", "ba"]:
        launch_file = "launch_ab.py" if config == "ab" else "launch_ba.py"
        try:
            cmd = ["rosrun", "monkeybot_autonomous", launch_file, "--length", str(length)]
            rospy.loginfo(f"Executing: {' '.join(cmd)}")
            subprocess.Popen(cmd)
        except Exception as e:
            rospy.logerr(f"Failed to launch {launch_file}: {e}")
    else:
        rospy.logerr("Failed to determine configuration")

if __name__ == "__main__":
    rospy.init_node('configuration_selector')
    configuration()
