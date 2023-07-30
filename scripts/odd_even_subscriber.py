#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32 ,String

def check_odd_or_even(number):
    return "odd" if number%2!=0 else "even"
def number_callback(data):
    result =check_odd_or_even(data.data)
    pub =rospy.Publisher("odd_even_result",String,queue_size=10)
    pub.publish(result)

if __name__=="__main__":
    rospy.init_node("odd_even_subscriber")
    rospy.Subscriber("random_number",Int32,number_callback)
    rospy.spin()