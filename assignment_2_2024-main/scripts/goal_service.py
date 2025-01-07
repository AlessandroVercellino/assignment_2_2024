#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *
import math

# Variabili globali
active_ = False
position_ = Point()
yaw_ = 0
state_ = 0
desired_position_ = Point()

# Precisione e parametri di controllo
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


def go_to_point_switch(req):
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res


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


def change_state(state):
    global state_
    state_ = state
    rospy.loginfo(f"State changed to [{state_}]")


def normalize_angle(angle):
    if math.fabs(angle) > math.pi:
        angle -= 2 * math.pi * angle / math.fabs(angle)
    return angle


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


def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0.0
    twist_msg.angular.z = 0.0
    pub.publish(twist_msg)


def main():
    global pub, active_, desired_position_

    rospy.init_node('go_to_point')

    # Publisher e Subscriber
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    # Servizio
    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)

    # Imposta valori predefiniti per i parametri
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

