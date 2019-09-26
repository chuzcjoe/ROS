#!/usr/bin/env python

"""
Use Keyboard to control the Baxter arms.

Zongcheng Chu
ECET 581
Lab1a
"""
import argparse

import rospy
from std_msgs.msg import String
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION


def talker():
    # function menu
    print("Use 1-6 to control the increment of left arm")
    print("Use Control + (1-6) to control the negative increment of left arm")
    print("Use q-y to control the right arm")
    print("Use Control + (q-y) to control the negative increment of right arm")
    
    #initialize node and topic
    pub = rospy.Publisher('ik',String,queue_size=10)
    rospy.init_node('ik_talker',anonymous=True)
    
    # mapping from key to function
    bindings = {
        'left_arm_increment': ['1','2','3','4','5','6'],
        'left_arm_decrement':  ['!','@','#','$','%','^'],
        'right_arm_increment': ['q','w','e','r','t','y'],
        'right_arm_decrement': ['Q','W','E','R','T','Y']
    }
    
    done = False
    message = ''
    while not done and not rospy.is_shutdown():
            c = baxter_external_devices.getch()
            if c:
                #catch Esc or ctrl-c
                if c in ['\x1b','\x03']:
                    done = True
                    rospy.signal_shutdown("Example finished.")
                elif c in bindings['left_arm_increment']:
                    message = "left_joint_increse "+ c
                elif c in bindings['left_arm_decrement']:
                    message = "left_joint_decrese "+ c
                elif c in bindings['right_arm_increment']:
                    message = "right_joint_increase "+ c
                elif c in bindings['right_arm_decrement']:
                    message = "right_joint_decrease "+ c
                # test begin
		elif c == 'z':
                    message = "left limb " + c 
                # test end
                else:
                    rospy.loginfo('key error!')
                
                if message != '':
                    #publish message to topic
                    rospy.loginfo(message)
                    pub.publish(message) 
                    
                
                    

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    
