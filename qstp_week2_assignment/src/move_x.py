#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_with_constant_velocity():
    rospy.init_node('move_turtlebot', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = 0.2  # Constant linear velocity
    
    while not rospy.is_shutdown():
        pub.publish(cmd_vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_with_constant_velocity()
    except rospy.ROSInterruptException:
        pass

