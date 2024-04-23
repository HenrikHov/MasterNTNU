#!/usr/bin/env python3
import rospy
import subprocess


def launch():
    config = "ab"
    while True:
        length = float(input("Enter a length between [-0.5, -1.2] or [0.5, 1.2]: "))
        if 0.5 <= abs(length) <=1.2:
            break
        else:
            rospy.loginfo("Input is not a valid value")

    rospy.loginfo(f"Launching with config: {config} and length: {length}")

    # Launch corresponding Python script based on configuration
    if config in ["ab", "ba"]:
        launch_file = "demo_launch_ab.py" if config == "ab" else "launch_ba.py"
        try:
            cmd = ["rosrun", "monkeybot_autonomous", launch_file, "--length", str(length)]
            rospy.loginfo(f"Executing: {' '.join(cmd)}")
            subprocess.Popen(cmd)
        except Exception as e:
            rospy.logerr(f"Failed to launch {launch_file}: {e}")
    else:
        rospy.logerr("Failed to determine configuration")

if __name__ == "__main__":
    rospy.init_node('autonomous_launcher')
    launch()

