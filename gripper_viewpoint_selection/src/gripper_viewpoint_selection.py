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

    # exit early if no contours found
    if(len(contours) == 0):
        return (None, None), (None, None, None, None)
    
    # find largest contour
    area_list = []
    for cnt in contours:
        area_list.append(cv2.contourArea(cnt))

    index_of_max = np.argmax(area_list)
    largest_contour = contours[index_of_max]
    
    # find target bounding box
    b_box = cv2.boundingRect(largest_contour)

    # extract center of b_box
    cx = (2*b_box[0] + b_box[2]) / 2
    cy = (2*b_box[1] + b_box[3]) / 2
    target_center = (cx, cy)

    return target_center, b_box

def in_buffer(center_point, buffer_bounds):
    cx, cy = center_point
    x, y, w, h = buffer_bounds

    # check if x and y coordinates are in bounds
    x_in_bounds = (cx > x) and (cx < x + w)
    y_in_bounds = (cy > y) and (cy < y + h)

    return (x_in_bounds and y_in_bounds)

def direction_to_move(center_point, buffer_bounds):
    cx, cy = center_point
    x, y, w, h = buffer_bounds
    direction_to_move_str = ["", ""]

    # obtain direction to move
    is_OOB_left = cx < x
    is_OOB_right = cx > x + w
    is_OOB_up = cy < y
    is_OOB_down = cy > y + h
    
    if(is_OOB_left):
        direction_to_move_str[0] = "right"
    elif(is_OOB_right):
        direction_to_move_str[0] = "left"
    if(is_OOB_up):
        direction_to_move_str[1] = "down"
    elif(is_OOB_down):
        direction_to_move_str[1] = "up"

    return direction_to_move_str   

def start_image_proc():
    # flag to indicate drawing over image
    IS_DRAWING = True

    # define buffer zone
    width = 400
    height = 200
    buffer_TL = (125, 125)
    buffer_BR = (buffer_TL[0] + width, buffer_TL[1] + height)
    buffer_bounds = (buffer_TL[0], buffer_TL[1], width, height)

    # move pan-tilt camera to initial position
    robot.camera.reset()
    init_camera_pose = [0.0, 0.35]
    robot.camera.set_pan_tilt(init_camera_pose[0], init_camera_pose[1], wait=True)

    while(True):
        try:
            # grab image
            rgb = robot.camera.get_rgb()
	    rgb = rgb[:, :, ::-1]
            img_copy = rgb.copy()

            # try to find target in image
	    target_center, b_box = find_target(rgb)

	    # skip this frame if target not found
            if(None in target_center or None in b_box):
                continue

            if(IS_DRAWING):
                # draw buffer area on image
                cv2.rectangle(img_copy, buffer_TL, buffer_BR, (255,0,255), 2)

                # draw target bounding box
                cv2.rectangle(img_copy, (b_box[0], b_box[1]), (b_box[0] + b_box[2], b_box[1] + b_box[3]), (255,0,0), 2)

                # draw marker on image center
                cv2.drawMarker(img_copy, target_center, (0,0,0), markerType=cv2.MARKER_CROSS, thickness=3)

            # check if detected center point is in buffer
            is_in_buffer = in_buffer(target_center, buffer_bounds)

            # if so, do nothing. Otherwise, move in direction it was last seen. 
	    if(is_in_buffer):
                pass
            else:
		dir_str = direction_to_move(target_center, buffer_bounds)

            cv2.imshow('Color', img_copy)
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
