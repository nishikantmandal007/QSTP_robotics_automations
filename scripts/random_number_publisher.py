#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Int32

def rand_dom ():
    return random.randint(1,100)

if __name__ == "__main__":
    rospy.init_node("random_number_publisher")
    pub = rospy.Publisher("random_number",Int32,queue_size=10)
    rate=rospy.Rate(1)

    while not rospy.is_shutdown():
        random_numbers = rand_dom()
        pub.publish(random_numbers)
        rate.sleep()
