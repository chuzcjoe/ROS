#!/usr/bin/env python

"""
Talker node keeps sending messages to the topic in a certain rate.

Zhiwen Cao, Zongcheng Chu
ECET 581
lab1 problem 2
"""

import rospy
from std_msgs.msg import String

def talker():
	#initialize node and topic
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
	
	#go through the loop 10 times per second
    rate = rospy.Rate(10) # 10hz
	
	#loop until the program exits
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass