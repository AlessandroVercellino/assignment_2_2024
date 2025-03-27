#!/usr/bin/env python

## @package assignment_2_2024-main
# \file goal_service.py
# \brief A service for a goal in Python
# \author Alessandro Vercellino 
# \version 1.0
# \date 15/03/2025
#
# \details 
#
# Publishes to: <BR>
# /cmd_vel
#
# Subcribes to: <BR>
# /odom
#
# Service : <BR> 
# /go_to_point_switch
#
#This script implements a simple go-to-point behavior for a mobile robot using 
#proportional control for both yaw and linear movement. The robot first aligns 
#its yaw to face the target position and then moves forward until it reaches 
#the desired position within a defined tolerance.

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math

# global variables
active_ = False
position_ = Point()
yaw_ = 0
state_ = 0
desired_position_ = Point()

# control parameters
yaw_precision_ = math.pi / 9  # +/- 20 degrees allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degrees allowed
dist_precision_ = 0.3

kp_a = 3.0
kp_d = 0.2
ub_a = 0.6
lb_a = -0.5
ub_d = 0.6

# Publisher
pub = None

##
# \brief Service callback 
# \param req Service request containing a boolean value
# \return Service response indicating success
#
# Service callback to enable or disable the go_to_point behavior
#
def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

##
# \brief Callback function for odometry data
# \param msg Odometry message containing the robot's position and orientation
#
# Callback function for odometry data which extracts and updates the robot's position and yaw angle from odometry data.
#
def clbk_odom(msg):
    global position_
    global yaw_

    # Aggiorna posizione
    position_ = msg.pose.pose.position

    # Aggiorna yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

##
# \brief Changes the state of the robot
# \param state New state to set
#
#  Updates the robot's current state and logs the change.
#
def change_state(state):
    global state_
    state_ = state
    rospy.loginfo(f"State changed to [{state_}]")

##
# \brief Normalizes an angle to be within [-pi, pi]
#
# \param angle Angle to normalize
#
# \return Normalized angle
#
#  Ensures that the angle remains within valid bounds to prevent erratic behavior.
#
def normalize_angle(angle):
    if math.fabs(angle) > math.pi:
        angle -= 2 * math.pi * angle / math.fabs(angle)
    return angle

##
# \brief Adjusts the yaw to face the desired position
#
# \param des_pos Target position
#
# Computes the desired yaw and applies proportional control to rotate towards it.
#
def fix_yaw(des_pos):
    global yaw_, pub, yaw_precision_2_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)

    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a * err_yaw
        twist_msg.angular.z = max(min(twist_msg.angular.z, ub_a), lb_a)

    pub.publish(twist_msg)

    if math.fabs(err_yaw) <= yaw_precision_2_:
        rospy.loginfo(f"Yaw error: [{err_yaw}]")
        change_state(1)

##
# \brief Moves the robot straight toward the desired position
#
# \param des_pos Target position
#
# Uses proportional control for forward motion while maintaining yaw alignment.
#
def go_straight_ahead(des_pos):
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = min(kp_d * err_pos, ub_d)
        twist_msg.angular.z = kp_a * err_yaw
        pub.publish(twist_msg)
    else:
        rospy.loginfo(f"Position error: [{err_pos}]")
        change_state(2)

    if math.fabs(err_yaw) > yaw_precision_:
        rospy.loginfo(f"Yaw error: [{err_yaw}]")
        change_state(0)

##
# \brief Stops the robot by setting velocities to zero
#
# Sends a zero-velocity command to halt the robot's movement.
#
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    pub.publish(twist_msg)

##
# \brief Main function to initialize the ROS node and control loop
#
# Initializes ROS node, sets up publishers, subscribers, services, and executes the control loop.
#
def main():
    global pub, active_, desired_position_

    rospy.init_node('go_to_point')

    # Publisher and Subscriber
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    # Service
    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    # Set standard values for parameters
    desired_position_.x = rospy.get_param('des_pos_x', 0.0)  # Default: 0.0
    desired_position_.y = rospy.get_param('des_pos_y', 0.0)  # Default: 0.0
    desired_position_.z = 0.0

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if not active_:
            rate.sleep()
            continue

        if state_ == 0:
            fix_yaw(desired_position_)
        elif state_ == 1:
            go_straight_ahead(desired_position_)
        elif state_ == 2:
            done()
        else:
            rospy.logerr("Unknown state!")

        rate.sleep()


if __name__ == '__main__':
    main()

