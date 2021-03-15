#!/usr/bin/env python
import rospy
from pyrobot import Robot
import time
import numpy as np
from telenursing_locobot_ctrl.srv import HomeArm, HomeArmResponse
from sensor_msgs.msg import Joy

# set joystick topic name
joyTopic = "/virtualJoystick"

def joyCallback(msg):
    """
    This function extracts the linear and angular command
    components from the recieved sensor_msgs/Joy message
    and sends them to the Locobot via an instance of the pyrobot 
    class.

    params:
        msg (sensor_msgs/Joy message): the recieved Joy message
    returns:
        None
    """
    global robot
    # extract message components
    arm_x = msg.axes[2]
    arm_y = msg.axes[3]
    arm_z = msg.axes[4]
    ee_yaw = msg.axes[5]
    ee_pitch = msg.axes[6]
    ee_roll = msg.axes[7]

    displacement = np.array([arm_x, arm_y, arm_z])
    success = robot.arm.move_ee_xyz(displacement, plan=True)
    rospy.loginfo("tried to move: ", success)

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

        # Initialize subscriber to respond to joystick inputs
        rospy.Subscriber(joyTopic, Joy, joyCallback)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
    rospy.spin()