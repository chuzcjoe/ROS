#!/usr/bin/env python

"""
Open an camera on baxter and also define the resolution of the camera opened.


Zhiwen Cao, Zongcheng Chu
ECET 581
lab1b Problem5
"""


import argparse

import rospy
from std_msgs.msg import String
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

from baxter_interface.camera import CameraController

#Open an specified camera with pre-defined resolution
def open_cam(camera, res):
	if not any((res[0] == r[0] and res[1] == r[1]) for r in CameraController.MODES):
		rospy.logerr("Invalid resolution provided.")
	cam = CameraController(camera)
	cam.resolution = res
	cam.open()

def talker():
	#initialize node and topic
    rospy.init_node('camera_control',anonymous=True)
	open_cam('left_hand_camera',(320,200)) #1280*800, 960*600, 640*400, 480*300, 384*240, 320*200
	


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
