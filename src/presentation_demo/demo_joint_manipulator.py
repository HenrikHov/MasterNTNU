#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import argparse
import numpy as np
from demo_ik_ab import inverse_kinematics

# Global variables
move_group = None
configuration = "ab"

def manipulate(end_state, input_length):
    joint_ik = inverse_kinematics(configuration, end_state, input_length)
    joint_state = np.array([joint_ik[0], 0.0, 0.0, joint_ik[1], joint_ik[2], 0.0, 0.0, joint_ik[3]])
    joint_goal = move_group.get_current_joint_values()
    for i, joint in enumerate(["joint0a", "joint1a", "joint2a", "joint3a", "joint3b", "joint2b", "joint1b", "joint0b"]):
        joint_index = move_group.get_active_joints().index(joint)
        joint_goal[joint_index] = joint_state[i]
    move_group.go(joint_goal, wait=True)
    move_group.stop()

def main(end_state, input_length):
    global move_group, configuration
    moveit_commander.roscpp_initialize(sys.argv)

    group_name = "arm"  # Replace with your move group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # First movement to the initial end state (home or dock)
    manipulate("home", input_length)
    manipulate("dock", input_length)
    rospy.loginfo("Docking completed")

if __name__ == '__main__':
    rospy.init_node('moveit_joint_control', anonymous=True)
    parser = argparse.ArgumentParser(description="Joint Manipulator")
    parser.add_argument("--end_state", type=str, default="home", help="End state: 'home' or 'dock'")
    parser.add_argument("--length", type=float, default=1.0, help="Length value")
    args = parser.parse_args()
    main(args.end_state, args.length)

