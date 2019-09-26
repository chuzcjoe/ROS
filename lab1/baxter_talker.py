#!/usr/bin/env python

"""
Use Keyboard to control the Baxter arms.
Use the keys 1-7 to control joints 1-7 on the left arm, with the corresponding shift keys causing negative
jogs, and keys q-u to control the right arm, with shift-Q – shift-U causing negative jogs.

Zongcheng Chu, Zhiwen Cao
ECET 581
Lab1a problem 3
"""
import argparse

import rospy
from std_msgs.msg import String
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION


def talker():
    # function menu
    print("Use 1-7 to control the increment of left arm")
    print("Use Control + (1-7) to control the negative increment of left arm")
    print("Use q-u to control the right arm")
    print("Use Control + (q-u) to control the negative increment of right arm")
    
    #initialize node and topic
    pub = rospy.Publisher('control',String,queue_size=10)
    rospy.init_node('talker',anonymous=True)
    
    # mapping from key to function
    bindings = {
        'left_arm_increment': ['1','2','3','4','5','6','7'],
        'left_arm_decrement':  ['!','@','#','$','%','^','&'],
        'right_arm_increment': ['q','w','e','r','t','y','u'],
        'right_arm_decrement': ['Q','W','E','R','T','Y','U']
    }
    
    done = False
    message = ''
    #Keep receiving key input
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
                    message = "left_joint_decrease "+ c
                    
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
    
