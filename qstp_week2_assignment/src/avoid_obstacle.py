#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry  # For odometry data

# Global variable to store the odometry data
odometry_data = None

def odometry_callback(data):
    global odometry_data
    odometry_data = data

def move_to_goal():
    rospy.init_node('move_to_goal', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odometry_callback)  # Subscribe to odometry topic

    goal_distance = 1.0  # Example goal distance in meters
    linear_velocity = 0.2
    goal_tolerance = 0.1  # Example tolerance in meters

    rate = rospy.Rate(10)
    cmd_vel_msg = Twist()

    while not rospy.is_shutdown():
        if odometry_data is not None:
            current_x = odometry_data.pose.pose.position.x
            current_distance = abs(current_x - goal_distance)
            
            if current_distance > goal_tolerance:
                cmd_vel_msg.linear.x = linear_velocity
            else:
                cmd_vel_msg.linear.x = 0.0

            pub.publish(cmd_vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_to_goal()
    except rospy.ROSInterruptException:
        pass

