#!/usr/bin/env python
import rospy
from pyrobot import Robot
import time
from telenursing_locobot_ctrl.srv import HomeArm, HomeArmResponse

def handle_home_arm(req):
    """
    Request handler for HomeArm service

    """
    global robot
    success = False
    if(req.home_locobot_arm): 
        success = robot.arm.go_home()

    return HomeArmResponse(success)

def main():
    # reference interface globally
    global robot
    arm_config = dict(control_mode='torque')
    # start robot control
    try:
        # Home robot arm
        robot = Robot('locobot', arm_config=arm_config)
        robot.arm.go_home()

        # initialize ROS service
        serviceHandler = rospy.Service("HomeArm", HomeArm, handle_home_arm)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
    rospy.spin()