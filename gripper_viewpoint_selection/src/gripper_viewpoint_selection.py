#!/usr/bin/env python

# Author: Trevor Sherrard
# Since: May 19, 2021
# Project: LoCoBot telenursing platform
# Purpose: to provide stable viewpoints of the gripper
#	   during the teloperation process

from pyrobot import Robot
from pyrobot.utils.util import try_cv2_import
import numpy as np
import rospy
import time

# Create the Robot object that interfaces with the robot.
robot = Robot('locobot')

# import opencv
cv2 = try_cv2_import()

# load image template
template = cv2.imread("../template/armarker.jpg", 0)

def find_target(image):
    target_center = (0,0)

    return target_center, image

def start_image_proc():
    while(True):
        try:
            # grab image
            rgb = robot.camera.get_rgb()
	    rospy.loginfo(rgb)
	    rgb = rgb[:, :, ::-1]

            # try to find target in image
	    target_center, image = find_target(rgb)
        
            cv2.imshow('Color', image)
            cmdChar = cv2.waitKey(1)
            if(cmdChar & 0xFF == ord('q')):
                break

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return

def main():
    # start selecting viewpoints
    start_image_proc()
    rospy.spin()

if(__name__ == "__main__"):
    main()
