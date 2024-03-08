#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import argparse
import numpy as np
from inverse_kinematics_ba import inverse_kinematics
from proximity_sensor_listener_ba import check_proximity  # Import proximity check function

# Global variables
move_group = None
configuration = "ba"

def manipulate(end_state, input_length):
    joint_ik = inverse_kinematics(configuration, end_state, input_length)
    joint_state = np.array([joint_ik[0], 0.0, 0.0, joint_ik[1], joint_ik[2], 0.0, 0.0, joint_ik[3]])
    joint_goal = move_group.get_current_joint_values()
    for i, joint in enumerate(["joint0b", "joint1b", "joint2b", "joint3b", "joint3a", "joint2a", "joint1a", "joint0a"]):
        joint_index = move_group.get_active_joints().index(joint)
        joint_goal[joint_index] = joint_state[i]
    move_group.go(joint_goal, wait=True)
    move_group.stop()



def main(end_state, input_length):
    global move_group, configuration
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_joint_control', anonymous=True)

    group_name = "arm"  # Replace with your move group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # First movement to the initial end state (home or dock)
    manipulate("home", input_length)

    # Check proximity before docking
    if check_proximity():
        rospy.loginfo("Safe to dock, proceeding with docking maneuver")

        # Update joint state for docking
        manipulate("dock", input_length)
        rospy.loginfo("Docking completed")
        rospy.signal_shutdown("Docking process completed successfully")
        
    else:
        rospy.loginfo("Not safe to dock, goes to other side")
        manipulate("home", -input_length)

        if check_proximity():
            rospy.loginfo("Safe to dock, proceeding with docking maneuver")

            # Update joint state for docking
            manipulate("dock", -input_length)
            rospy.loginfo("Docking completed")
            rospy.signal_shutdown("Docking process completed successfully")

        else:
            rospy.loginfo("No safe place to land with provided length value. Try again with different a value")



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Joint Manipulator")
    parser.add_argument("--end_state", type=str, default="home", help="End state: 'home' or 'dock'")
    parser.add_argument("--length", type=float, default=1.0, help="Length value")
    args = parser.parse_args()
    main(args.end_state, args.length)
