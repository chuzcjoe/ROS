#!/usr/bin/env python

"""
Receive the commmand from talker and move the corresponding joint.

Zongcheng Chu, Zhiwen Cao
ECET 581
Lab1a problem3
"""
import argparse

import rospy
from std_msgs.msg import String

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION



#Function of control baxter arms
def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)


def callback(data):
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    #get robot state
    init_state = rs.state().enabled
    #enable robot
    rs.enable()
    
    rospy.loginfo(rospy.get_caller_id() + "I heard %s",data.data)
    message = data.data
    
    #Get baxter arm and joint names
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    lj = left.joint_names()
    rj = right.joint_names()
    
    li = ['1','2','3','4','5','6','7']
    ld = ['!','@','#','$','%','^','&']
    ri = ['q','w','e','r','t','y','u']
    rd = ['Q','W','E','R','T','Y','U']
    
    #control arm movement in terms of the message received
    if message[-1] in li:
        set_j(left,lj[li.index(message[-1])],0.2)
    elif message[-1] in ld:
        set_j(left,lj[ld.index(message[-1])],-0.2)
    elif message[-1] in ri:
        set_j(right,rj[ri.index(message[-1])],0.2)
    elif message[-1] in rd:
        set_j(right,rj[rd.index(message[-1])],-0.2)
    else:
        pass
    
    

def listener():
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber('control',String,callback)
    
    rospy.spin()   

if __name__ == '__main__':
    listener()
