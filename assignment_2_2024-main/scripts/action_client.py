#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from assignment_2_2024.msg import PlanningAction, PlanningGoal  # Import the custom action messages

# Feedback callback function
def feedback_callback(feedback):
    """
    Callback to handle feedback from the Action Server.
    """
    rospy.loginfo(f"Feedback received: Current Position: {feedback.actual_pose}, Status: {feedback.stat}")

# Function to send a goal to the Action Server
def send_goal(client, x, y):
    """
    Sends a goal to the Action Server.

    Args:
        client (SimpleActionClient): The Action Client instance.
        x (float): The x-coordinate of the goal.
        y (float): The y-coordinate of the goal.
    """
    goal = PlanningGoal()
    goal.target_pose.header.frame_id = "map"  # Set the reference frame
    goal.target_pose.header.stamp = rospy.Time.now()  # Timestamp for the goal
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0  # 2D robot
    goal.target_pose.pose.orientation.w = 1.0  # Default orientation (no rotation)

    # Send the goal to the Action Server
    client.send_goal(goal, feedback_cb=feedback_callback)
    rospy.loginfo(f"Goal sent: x={x}, y={y}")

# Function to cancel the current goal
def cancel_goal(client):
    """
    Cancels the current goal being processed by the Action Server.

    Args:
        client (SimpleActionClient): The Action Client instance.
    """
    rospy.loginfo("Cancelling the current goal...")
    client.cancel_goal()
    rospy.sleep(0.5)  # Allow time for the cancel request to propagate
    state = client.get_state()
    if state in [actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.RECALLED]:
        rospy.loginfo("Goal successfully cancelled.")
    else:
        rospy.logwarn("Failed to cancel the goal.")

# Function to get user input for goal coordinates
def get_input():
    """
    Prompts the user to input the x and y coordinates for the goal.

    Returns:
        tuple: A tuple containing the x and y coordinates.
    """
    while True:
        try:
            x = float(input("Enter the x-coordinate for the goal: "))
            y = float(input("Enter the y-coordinate for the goal: "))
            return x, y
        except ValueError:
            rospy.logwarn("Invalid input! Please enter numeric values only.")

def main():
    """
    Main function to initialize the Action Client and handle user commands.
    """
    rospy.init_node('action_client')

    # Create an Action Client for the /reaching_goal Action Server
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    rospy.loginfo("Waiting for the Action Server to become available...")
    client.wait_for_server()

    rospy.loginfo("Action Client is ready!")
    rospy.loginfo("Commands: 's' (send goal), 'c' (cancel goal), 'q' (quit)")

    while not rospy.is_shutdown():
        command = input("Enter command ('s'=send goal, 'c'=cancel goal, 'q'=quit): ").strip().lower()

        if command == 's':  # Send a goal
            x, y = get_input()
            send_goal(client, x, y)
        elif command == 'c':  # Cancel the current goal
            cancel_goal(client)
        elif command == 'q':  # Quit the Action Client
            rospy.loginfo("Exiting the Action Client.")
            break
        else:
            rospy.logwarn("Invalid command. Please try again.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted.")

