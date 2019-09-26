#!/usr/bin/env python

"""
Listener captures each frame from the camera scene and convert it into OpenCV image format.
SSD (sum of squared differences) is applied to get the pattern location and move the real Baxter robot to track a moving
object in Cartesian space.

Zhiwen Cao, Zongcheng Chu
ECET 581
lab1b Problem5
"""

import argparse
import struct
import sys
import baxter_interface
import baxter_external_devices
import math

import tf
import sys

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

import cv2, cv_bridge
from sensor_msgs.msg import Image

#Conversion factors
factor_x =  11.0/12.0 * 0.001
factor_y =  11.0/12.0 * 0.001


#find the location where mat2 should be in mat1 by computing the least sum of squared difference
def findMat(mat1,mat2):
	error = 0
	loc = [0,0]

	end_r = mat1.shape[1] - mat2.shape[1]
	end_b = mat1.shape[0] - mat2.shape[0]

	MIN = sys.maxint
	for i in range(0,end_b+1):
		print(i)
		for j in range(0,end_r+1):
			for m in range(0,mat2.shape[0]):
				for n in range(0,mat2.shape[1]):
					for ch in range(0,3):
						error = error + (mat2[m,n,ch] - mat1[i+m-1,j+n-1,ch]) ** 2
			if error < MIN:
				MIN = error
				loc[0] = i
				loc[1] = j

			error = 0
	return loc
				

	

# Callback function to subscribe to images
def image_callback(ros_img):
	
	
	pattern = cv2.imread("/home/crl7/ros_ws/marker.jpg")
	rs = baxter_interface.RobotEnable(CHECK_VERSION)
        #get robot state
        init_state = rs.state().enabled
        #enable robot
        rs.enable()
	
	
	#left arm initial state
	left = baxter_interface.Limb('left')
    	initial_left = left.endpoint_pose()['position'][:] + left.endpoint_pose()['orientation'][:]
    	Pose_state_left = [x for x in initial_left]
    	limb = 'left'
	
	
	ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	
	
	bridge = cv_bridge.CvBridge()
	# Convert received image message to OpenCv image
	cv_image = bridge.imgmsg_to_cv2(ros_img, desired_encoding="passthrough")
	#print(cv_image.shape)  #h, w , c
	print("convert finished.")
	
	#get the camera view center.
	center_x = cv_image.shape[1] // 2
	center_y = cv_image.shape[0] // 2

	
	lo = findMat(cv_image,pattern)

	cur_x = lo[1] + pattern.shape[1] // 2	
	cur_y = lo[0] + pattern.shape[0] // 2
	print(cur_x, cur_y)

	#the x and y distance that the camera should move in order to put the pattern in center of the camera filed of view.
	df_x = cur_x - center_x
	df_y = cur_y - center_y

	#keep the rotation pose the same, only change the x and y position
	poses = {
            'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=Pose_state_left[0] + df_y*factor_y,
                    y=Pose_state_left[1] + df_x*factor_x,
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
    	
        	left.set_joint_positions(joint_command)
    	
    	### test end
    	##################################
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

		#--------show imag---------
	
	
	cv2.imshow('Image', cv_image)
	#cv2.imwrite('/home/crl7/ros_ws/pattern.jpg',cv_image)   
	# display image
	cv2.waitKey(1)
	


def listener():
    rospy.init_node('camera_listener',anonymous=True)
    rospy.Subscriber('/cameras/left_hand_camera/image', Image, image_callback)
    
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    listener()

