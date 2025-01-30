#!/usr/bin/env python

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, Twist
from assignment_2_2024.msg import PlanningAction, PlanningGoal  # Import the custom action messages
from std_srvs.srv import Trigger, TriggerResponse

# Global variables to track goals reached and canceled
goals_reached = 0
goals_canceled = 0

# Publisher for robot velocity
velocity_publisher = None

def feedback_callback(feedback):
    """
    Callback to handle feedback from the Action Server.
    """
    rospy.loginfo(f"Feedback received: Current Position: {feedback.actual_pose}, Status: {feedback.stat}")

# Function to send a goal to the Action Server
def send_goal(client, x, y):
    """
    Sends a goal to the Action Server.
    """
    global goals_reached
    goal = PlanningGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal, feedback_cb=feedback_callback)
    rospy.loginfo(f"Goal sent: x={x}, y={y}")
    client.wait_for_result()
    result = client.get_state()
    if result == actionlib.GoalStatus.SUCCEEDED:
        goals_reached += 1
        rospy.loginfo("Goal reached successfully.")
    else:
        rospy.logwarn("Goal failed to be reached.")

# Function to cancel the current goal
def cancel_goal(client):
    """
    Cancels the current goal being processed by the Action Server.
    """
    global goals_canceled
    rospy.loginfo("Cancelling the current goal...")
    client.cancel_goal()
    rospy.sleep(0.5)
    state = client.get_state()
    if state in [actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.RECALLED]:
        goals_canceled += 1
        rospy.loginfo("Goal successfully cancelled.")
    else:
        rospy.logwarn("Failed to cancel the goal.")

# Service callback function to return the number of goals reached and canceled
def goal_status_service_callback(request):
    """
    Returns the number of goals reached and canceled.
    """
    response = TriggerResponse()
    response.success = True
    response.message = f"Goals reached: {goals_reached}, Goals canceled: {goals_canceled}"
    return response

# Function to publish velocity in km/h
def velocity_callback(msg):
    """
    Converts and publishes velocity in km/h.
    """
    global velocity_publisher
    velocity_kmh = Twist()
    velocity_kmh.linear.x = msg.linear.x * 3.6
    velocity_kmh.linear.y = msg.linear.y * 3.6
    velocity_publisher.publish(velocity_kmh)

# Function to get user input for goal coordinates
def get_input():
    while True:
        try:
            x = float(input("Enter the x-coordinate for the goal: "))
            y = float(input("Enter the y-coordinate for the goal: "))
            return x, y
        except ValueError:
            rospy.logwarn("Invalid input! Please enter numeric values only.")

# Main function
def main():
    rospy.init_node('action_client')
    global velocity_publisher

    # Create an Action Client for the /reaching_goal Action Server
    client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    rospy.loginfo("Waiting for the Action Server to become available...")
    client.wait_for_server()
    rospy.loginfo("Action Client is ready!")
    
    # Initialize the velocity publisher
    velocity_publisher = rospy.Publisher('/robot_velocity_kmh', Twist, queue_size=10)
    rospy.Subscriber('/robot_velocity', Twist, velocity_callback)
    
    # Initialize the service
    rospy.Service('/goal_status', Trigger, goal_status_service_callback)
    rospy.loginfo("Service /goal_status is ready.")

    rospy.loginfo("Commands: 's' (send goal), 'c' (cancel goal), 'q' (quit)")
    while not rospy.is_shutdown():
        command = input("Enter command ('s'=send goal, 'c'=cancel goal, 'q'=quit): ").strip().lower()

        if command == 's':
            x, y = get_input()
            send_goal(client, x, y)
        elif command == 'c':
            cancel_goal(client)
        elif command == 'q':
            rospy.loginfo("Exiting the Action Client.")
            break
        else:
            rospy.logwarn("Invalid command. Please try again.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted.")

