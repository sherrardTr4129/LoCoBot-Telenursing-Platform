#!/usr/bin/env python

import rospy
from pyrobot import Robot
import numpy as np
from geometry_msgs.msg import Twist, PointStamped
from tn_locobot_control.srv import pointAndClick, pointAndClickResponse

pointAndClickServiceName = "PointAndClickSrv"

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

    # get real world points in robot base frame
    pt, color = robot.camera.pix_to_3dpt(req.row, req.col)

    # publish point for rviz visualization
    #testPt = PointStamped()
    #testPt.point.x = pt[0][0]
    #testPt.point.y = pt[0][1]
    #testPt.point.z = pt[0][2]
    #testPt.header.stamp = rospy.Time.now()
    #testPt.header.frame_id = "base_link"
    #pubTest.publish(testPt)

    # print recieved point
    rospy.loginfo('got point %s' % pt)

    # try to move to point
    #robot.base.go_to_relative([pt[0][0], pt[0][1], 0.0], use_map=False, smooth=False, close_loop=True)

    return pointAndClickResponse(True)

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
