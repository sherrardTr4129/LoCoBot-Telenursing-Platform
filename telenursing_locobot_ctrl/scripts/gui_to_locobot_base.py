#!/usr/bin/env python

import rospy
from pyrobot import Robot
import time
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

    # extract message components
    fwdRev = msg.axes[0]
    spin = msg.axes[1]

    # halve each value - need to verify whether this is needed for Locobot or is Trina-specific
    if(fwdRev != 0):
        fwdRev /= 2
    if(spin != 0):
        spin /= 2

    # invert the direction of the scaled spin value - need to verify whether this is needed for Locobot or is Trina-specific
    spin = -spin

    # Pass command to robot base
    execution_time = 1   # 1 second per command, for now
    robot = Robot('locobot') 
    robot.base.set_vel(fwd_speed=fwdRev, 
                   turn_speed=spin, 
                   exe_time=execution_time)


if __name__ == "__main__":
    try:
        rospy.init_node("gui_to_base_control")
        rospy.Subscriber(joyTopic, Joy, joyCallback)
    except rospy.ROSInterruptException:
        pass
    
    rospy.spin()