#!/usr/bin/env python
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import radians

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_joint_control', anonymous=True)

    # Replace "arm" with your actual move group name
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Specify the joint you wish to move
    joint_name = "joint3b"  # Ensure this matches the actual joint name in your robot

    while not rospy.is_shutdown():
        try:
            desired_angle_deg = float(input(f"Enter the desired angle for {joint_name} in degrees: "))
            desired_angle_rad = radians(desired_angle_deg)

            # Get current joint positions
            joint_goal = move_group.get_current_joint_values()

            # TODO: Update the index of joint_goal to match the position of your joint
            joint_index = move_group.get_active_joints().index(joint_name)
            joint_goal[joint_index] = desired_angle_rad

            # Move to the new joint position
            move_group.go(joint_goal, wait=True)

            # Ensure no further movement
            move_group.stop()
        except ValueError:
            print("Invalid input, please enter a number.")
        except IndexError:
            print(f"Joint {joint_name} not found in the active joints of the group {group_name}.")

if __name__ == '__main__':
    main()
