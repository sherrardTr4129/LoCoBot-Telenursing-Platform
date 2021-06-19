#!/usr/bin/env python

import rospy
from pyrobot import Robot
import numpy as np
from geometry_msgs.msg import Twist, PointStamped
from tn_locobot_control.srv import pointAndClick, pointAndClickResponse

# define globals
pointAndClickServiceName = "PointAndClickSrv"
testPtTopic = "/testPoint"
FLOOR_THRESH = 0.23

# create publisher for test point
pubTest = rospy.Publisher(testPtTopic, PointStamped, queue_size=10)

def gen_random_pt(UL, BR, num_samples):
    """
    generate a list of random (row, col) tuples within the
    bbox defined by UL and BR

    params:
	UL (row, col): upper left point of bbox
        BR (row, col): bottom right point of bbox
        num_samples (int): number of samples to cast.

    returns:
        ptList ([(row, col)]): random pt samples within ROI
    """

    ptList = []
    for i in range(0, num_samples):
        rand_row = np.random.randint(UL[0], BR[0])
        rand_col = np.random.randint(UL[1], BR[1])
        ptList.append((rand_row, rand_col))
        
    return ptList

def isPointOnFloor(point, frame, floor_thresh):
    """
    this function verifies that the point selected by a user
    (and a suitable area around said point) is on a flat surface.

    params:
        point ((row, col)): user selected row and col within image
    returns:
        isOnFloor (bool): a boolean indicating whether point is on floor or not.
    """
    global robot

    # set up local variables
    isOnFloor = False
    NUM_SAMPLES = 100
    ROI_H = 50
    ROI_W = 50

    # obtain ROI from selected point
    UL_col = point[1] - (ROI_H/2)
    UL_row = point[0] - (ROI_W/2)
    
    # constrain UL coordinates to positive values
    if(UL_row < 0):
        UL_row = 0
    if(UL_col < 0):
        UL_col = 0

    # create BR point from UL point
    BR_row = UL_row + ROI_H
    BR_col = UL_col + ROI_W

    # constrain BR coordinates to max image size
    max_row, max_col, ch = frame.shape
    if(BR_row > max_row):
        BR_row = max_row
    if(BR_col > max_col):
        BR_col = max_col

    # generate random pixel samples within ROI
    UL = (UL_row, UL_col)
    BR = (BR_row, BR_col)
    ptList = gen_random_pt(UL, BR, NUM_SAMPLES)

    ROI_Z_vals = []
    # loop ROI samples, extract z-value at each location.
    for point in ptList:
        (row, col) = point
        pt, _ = robot.camera.pix_to_3dpt(row, col)
        z_val = pt[0][-1]
        ROI_Z_vals.append(z_val)

    # compute average z-value
    avg_z = sum(ROI_Z_vals)/len(ROI_Z_vals)

    # check average z value against floor_tresh
    if(avg_z > floor_thresh):
        isOnFloor = False
    else:
        isOnFloor = True

    return isOnFloor

def PointAndClickService(req):
    """
    This function serves as the service callback to extract real-world
    coordinates from data obtained from user point and click interface

    params:
	req(pointAndClick instance): the pointAndClick request
    returns:
        response (pointAndClickResponse instance): the service result
    """
    global robot

    # grab frame from camera
    frame = robot.camera.get_rgb()

    # verify point is on floor
    isOnFloor = isPointOnFloor((req.row, req.col), frame, FLOOR_THRESH)

    if(isOnFloor):
        # get real world points in robot base frame
        pt, color = robot.camera.pix_to_3dpt(req.row, req.col)

        # publish point for rviz visualization
        testPt = PointStamped()
        testPt.point.x = pt[0][0]
        testPt.point.y = pt[0][1]
        testPt.point.z = pt[0][2]
        testPt.header.stamp = rospy.Time.now()
        testPt.header.frame_id = "map"
        pubTest.publish(testPt)

        # print recieved point
        rospy.loginfo('got point %s' % pt)

        # try to move to point
        #robot.base.go_to_relative([pt[0][0], pt[0][1], 0.0], use_map=False, smooth=False, close_loop=True)

        # return status
        return pointAndClickResponse(isOnFloor)

    else:
        return pointAndClickResponse(isOnFloor)

def main():
    # reference globals
    global robot

    # define control modes
    arm_config = dict(control_mode='torque')
    base_config_dict = {'base_controller': 'proportional', 'base_planner':'movebase'}

    try:
        # init robot instance
        robot = Robot('locobot_cam', arm_config=arm_config, base_config=base_config_dict)
        rospy.loginfo('robot initialized in stream manager!')

        # start service listener
        pointAndClickSrv = rospy.Service(pointAndClickServiceName, pointAndClick, PointAndClickService)
        rospy.loginfo('point and click service is registered. ready for requests!')

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if(__name__ == "__main__"):
    main()
    rospy.spin()
