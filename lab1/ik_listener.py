#!/usr/bin/env python

"""
Lab1b problem 4:
    Use Baxter’s inverse kinematics (IK) to do the menu-based jogging as problem 3.
Zongcheng Chu, Zhiwen Cao
ECET 581
"""

import argparse
import struct
import sys
import baxter_interface
import baxter_external_devices
import math

import tf

from baxter_interface import CHECK_VERSION
import rospy
from std_msgs.msg import String

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)



def callback(data):
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    #get robot state
    init_state = rs.state().enabled
    #enable robot
    rs.enable()

    # get the left and right arm
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    # get the current pose information of left and right arm
    initial_left = left.endpoint_pose()['position'][:] + left.endpoint_pose()['orientation'][:]
    initial_right = right.endpoint_pose()['position'][:] + right.endpoint_pose()['orientation'][:]

    # create a list for each limb for updating
    Pose_state_left = [x for x in initial_left]
    Pose_state_right = [x for x in initial_right]

    message = data.data
    limb = ''
    
    # message[0] to decide whether left or right limb
    if message[0] == 'l':
        limb = 'left'
    
    if message[0] == 'r':
        limb = 'right'
    
    # invoke Baxter's inverse kinematic service.
    # rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    
    # the lists for deciding the increasing / decreasing of left / right limbs
    li = ['1','2','3','4','5','6']
    ld = ['!','@','#','$','%','^']
    ri = ['q','w','e','r','t','y']
    rd = ['Q','W','E','R','T','Y']
    
    # initialization of quaternion elements
    q_w = q_x = q_y = q_z = 0.0

    # left limb
    if message[0] == 'l':
        # left limb increasement
        if message[-1] in li:
            idx = li.index(message[-1])
            # update x,y,z 
            if idx in [0,1,2]:
                # add 0.01m to current x/y/z
                Pose_state_left[idx] += 0.01
            # update raw, pitch, yaw
            if idx in [3,4,5]:
                # transform from current quaterion to euler angles in radians by x-y-z order
		        Euler = tf.transformations.euler_from_quaternion( (Pose_state_left[3],Pose_state_left[4],Pose_state_left[5],Pose_state_left[6]) )
		        Euler = [x for x in Euler]
		        # add 0.1 radians to current euler angles
		        Euler[idx-3] += 0.1
                # transform back from euler angles to quaternions by x-y-z order
                Pose_state_left[3],Pose_state_left[4],Pose_state_left[5],Pose_state_left[6] = tf.transformations.quaternion_from_euler(Euler[0],Euler[1],Euler[2])
            poses = {
                'left': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=Pose_state_left[0],
                        y=Pose_state_left[1],
                        z=Pose_state_left[2],
                    ),
                orientation=Quaternion(
                    x=Pose_state_left[3],
                    y=Pose_state_left[4],
                    z=Pose_state_left[5],
                    w=Pose_state_left[6])))
                    }
        # left limb decreasement
        elif message[-1] in ld:
            idx = ld.index(message[-1])
            # x,y,z update
            if idx in [0,1,2]:
                Pose_state_left[idx] -= 0.01
            # row, pitch, yaw update
            if idx in [3,4,5]:
        
        # transform quaternion to euler angles
		Euler = tf.transformations.euler_from_quaternion( (Pose_state_left[3],Pose_state_left[4],Pose_state_left[5],
					Pose_state_left[6]) )
		Euler = [x for x in Euler]
		Euler[idx-3] -= 0.1
		print("Euler:----",Euler)
                # transform euler angles to quaternions
                Pose_state_left[3],Pose_state_left[4],Pose_state_left[5],Pose_state_left[6] = tf.transformations.quaternion_from_euler(Euler[0],Euler[1],Euler[2])
            poses = {
            'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=Pose_state_left[0],
                    y=Pose_state_left[1],
                    z=Pose_state_left[2],
                ),
                orientation=Quaternion(
                    x=Pose_state_left[3],
                    y=Pose_state_left[4],
                    z=Pose_state_left[5],
                    w=Pose_state_left[6],
                ),
            ),
        )
    }
        # this part is for problem 05
        # Set to a specific position, where the camera points straight down to the ground
        # and x-axis aligned with y-axis
        elif message[-1] == 'z':
            # we set raw = 180, pitch = 0, yaw = 0
            quad = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
            poses = {
            'left': PoseStamped(
                    header=hdr,
                    pose=Pose(
                        # that's the x-y-z coordinates we set in Cartesian world.
                        position=Point(
                            x=0.26614726495824964,    
                            y=0.6360726991380656,
                            z=-0.6312665419779635,
			    
                        ),
                        orientation=Quaternion(
                            x=quad[0],
                            y=quad[1],
                            z=quad[2],
                            w=quad[3],
                        ),
                    ),
                )
            }
        # test end

    
    if message[0] == 'r':
        if message[-1] in ri:
            idx = ri.index(message[-1])
            if idx in [0,1,2]:
                Pose_state_right[idx] += 0.01
            if idx in [3,4,5]:
		Euler = tf.transformations.euler_from_quaternion( (Pose_state_right[3],Pose_state_right[4],Pose_state_right[5],
					Pose_state_right[6]) )
		Euler = [x for x in Euler]
		Euler[idx-3] += 0.01
		print("Euler:----",Euler)
                
                Pose_state_right[3],Pose_state_right[4],Pose_state_right[5],Pose_state_right[6] = tf.transformations.quaternion_from_euler(Euler[0],Euler[1],Euler[2])
            
            poses = {
            'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=Pose_state_right[0],
                    y=Pose_state_right[1],
                    z=Pose_state_right[2],
                ),
                orientation=Quaternion(
                    x=Pose_state_right[3],
                    y=Pose_state_right[4],
                    z=Pose_state_right[5],
                    w=Pose_state_right[6],
                ),
            ),
        )
    }
        
        elif message[-1] in rd:
            idx = rd.index(message[-1])
            if idx in [0,1,2]:
                Pose_state_right[idx] -= 0.01
            if idx in [3,4,5]:
		Euler = tf.transformations.euler_from_quaternion( (Pose_state_right[3],Pose_state_right[4],Pose_state_right[5],
					Pose_state_right[6]) )
		Euler = [x for x in Euler]
		Euler[idx-3] -= 0.01
		print("Euler:----",Euler)
                
                Pose_state_right[3],Pose_state_right[4],Pose_state_right[5],Pose_state_right[6] = tf.transformations.quaternion_from_euler(Euler[0],Euler[1],Euler[2])
            poses = {
            'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=Pose_state_right[0],
                    y=Pose_state_right[1],
                    z=Pose_state_right[2],
                ),
                orientation=Quaternion(
                    x=Pose_state_right[3],
                    y=Pose_state_right[4],
                    z=Pose_state_right[5],
                    w=Pose_state_right[6],
                ),
            ),
        )
    }
    
    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp


        ##################################
        ### apply new positions to joints
        joint_command = limb_joints
        if message[0] == 'l':
            left.set_joint_positions(joint_command)
        if message[0] == 'r':
            right.set_joint_positions(joint_command)
        ### test end
        ##################################
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0
    
        
            


def listener():
    rospy.init_node('ik_listener',anonymous=True)
    rospy.Subscriber('ik',String,callback)
    
    rospy.spin() 

if __name__ == '__main__':
    listener()
