#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
from math import sin, cos

# List of goals that will be sent to the robot
goal_list = []

# Global index for the current goal
current_goal_index = 0

# Callback function for the /move_base/status subscriber
def status_callback(status_msg):
    global current_goal_index

    # Check if there is at least one status in the list
    if status_msg.status_list:
        last_status = status_msg.status_list[-1]  # Get the last status

        # If the last goal succeeded (status code 3), send the next goal
        if last_status.status == 3:  # SUCCEEDED
            current_goal_index += 1

            if current_goal_index < total_goals:
                rospy.loginfo(f"Goal succeeded, sending next goal ({current_goal_index + 1}/{total_goals}).")
                send_new_goal(current_goal_index)
            else:
                rospy.loginfo("All goals have been sent and reached successfully.")

# Function to send a new goal to the /move_base/goal topic
def send_new_goal(goal_index):
    goal = MoveBaseGoal()

    # Define the goal position and orientation (coordinates from goal_list)
    goal_vct = goal_list[goal_index]
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_vct[0]
    goal.target_pose.pose.position.y = goal_vct[1]
    if(len(goal_vct) == 3):
        # goal_vct = [x, y, θ (rad)]. convert θ to qz, qw.
        goal.target_pose.pose.orientation.z = sin(goal_vct[2]*0.5)
        goal.target_pose.pose.orientation.w = cos(goal_vct[2]*0.5)
    elif(len(goal_vct) == 4):
        # goal_vct = [x, y, qz, qw]
        goal.target_pose.pose.orientation.z = goal_vct[2]
        goal.target_pose.pose.orientation.w = goal_vct[3]

    # Send goal
    client.send_goal(goal)
    rospy.loginfo(f"New goal sent: x = {goal_vct[0]}, y = {goal_vct[1]} ({goal_index + 1}/{total_goals})")

if __name__ == "__main__":
    rospy.init_node('goal_publisher')

    # parse the ros parameters. eval() evaluates a string with an expression as python code
    start_delay = int(rospy.get_param('~start_delay', 0))
    goal_list = eval(rospy.get_param('~goal_list', []))

    total_goals = len(goal_list)

    # sleeping so we have time to set the pose in rviz, for lab runs
    rospy.sleep(start_delay)

    # Create a SimpleActionClient for the move_base action server
    # ... and wait for the action server to become available
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to move_base action server")

    # Subscribe to the /move_base/status topic
    rospy.Subscriber('/move_base/status', GoalStatusArray, status_callback)

    # Send the first goal initially
    rospy.loginfo(f"Sending the first goal (1/{total_goals})")
    send_new_goal(current_goal_index)

    # Keep the node running
    rospy.spin()
