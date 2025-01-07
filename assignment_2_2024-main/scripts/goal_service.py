#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerResponse

# Global variable to store the last goal
last_goal = None

def handle_service(req):
    """
    Handles requests to the /get_last_goal service.

    Args:
        req: The service request (not used in Trigger).

    Returns:
        TriggerResponse: Contains success status and the last goal or an error message.
    """
    if last_goal:
        return TriggerResponse(
            success=True,
            message=f"Last goal: x={last_goal[0]}, y={last_goal[1]}"
        )
    else:
        return TriggerResponse(
            success=False,
            message="No goal has been set yet."
        )

def update_last_goal(x, y):
    """
    Updates the global last_goal variable with new coordinates.

    Args:
        x (float): x-coordinate of the goal.
        y (float): y-coordinate of the goal.
    """
    global last_goal
    last_goal = (x, y)
    rospy.loginfo(f"Last goal updated: x={x}, y={y}")

def main():
    """
    Main function to initialize the node and the service.
    """
    rospy.init_node('goal_service')

    # Create the service
    service = rospy.Service('get_last_goal', Trigger, handle_service)
    rospy.loginfo("Goal Service Node started. Ready to provide the last goal.")

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logerr("Program interrupted.")

