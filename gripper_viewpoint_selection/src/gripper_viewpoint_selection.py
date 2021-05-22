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

# import opencv through pyRobot
cv2 = try_cv2_import()

def find_target(image):
    lower_lim = np.array([66,87,200])
    upper_lim = np.array([255,255,255])
    target_center = (0,0)

    # make image copy
    img_copy = image.copy()

    # blur image
    image_blurred = cv2.blur(image, (5,5))

    # convert to HSV color space
    hsv_image = cv2.cvtColor(image_blurred, cv2.COLOR_RGB2HSV)

    # create orange mask
    orange_mask = cv2.inRange(hsv_image, lower_lim, upper_lim)

    # dilate mask image
    kernel = np.ones((7,7), np.uint8)
    orange_mask_dialated = cv2.dilate(orange_mask, kernel, iterations=2)

    # find contours in image
    _, contours, _ = cv2.findContours(orange_mask_dialated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # draw contours on original image
    cv2.drawContours(img_copy, contours, -1, (0, 255, 0), 2)

    return target_center, img_copy

def start_image_proc():
    while(True):
        try:
            # grab image
            rgb = robot.camera.get_rgb()
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
