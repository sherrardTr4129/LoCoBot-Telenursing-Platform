#!/usr/bin/env python
import rospy
from pyrobot import Robot
import time
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int8

# define misc variables
namespace = "/pyRobot_web_data/"
twistTopic = "cmd_vel"
gripperStateTopic = "gripper_state"
panTiltOffsetTopic = "pan_tilt_offset"
xyzArmOffsetTopic = "xyz_arm_offet"

def xyzArmCallback(msg):
    """
    This function extracts the xyz components of the Float32 array
    message request for cartesian end effector movement, repackages
    as a Numpy array, and sends them to the Locobot via an instance 
    of the pyrobot class.  Execution time is 0.5 seconds.  Planning
    uses pyrobot's built-in inverse kinematics with no collision 
    avoidance.

    params:
        msg (std_msgs/Float32MultiArray): the recieved message
    returns:
        None
    """
    global robot
    # extract message components
    arm_x = msg.data[0]
    arm_y = msg.data[1]
    arm_z = msg.data[2]

    if (arm_x == 0 and arm_y == 0 and arm_z == 0):
        rospy.loginfo("no arm movement requested")
    
    else:
        displacement = np.array([arm_x, arm_y, arm_z])
        success = robot.arm.move_ee_xyz(displacement, plan=False)
        rospy.loginfo("tried to move arm")
 
def twistCallback(msg):
    """
    This function extracts the linear and angular command
    components from the recieved Twist message
    and sends them to the Locobot via an instance of the pyrobot 
    class.

    params:
        msg (geometry_msgs/Twist message): the recieved message
    returns:
        None
    """
    global robot
    # extract message components
    fwdRev = msg.linear.x
    spin = msg.angular.z

    # halve each value - need to verify whether this is needed for Locobot or is Trina-specific
    if(fwdRev != 0):
        fwdRev /= 2
    if(spin != 0):
        spin /= 2

    # invert the direction of the scaled spin value - need to verify whether this is needed for Locobot or is Trina-specific
    spin = -spin

    # Pass command to robot base
    execution_time = 0.5   # match with web command refresh rate
     
    robot.base.set_vel(fwd_speed=fwdRev, 
                   turn_speed=spin, 
                   exe_time=execution_time)

def gripperCallback(msg):
    """
    This function calls the open and close gripper routines...

    params:
        msg (std_msgs/Int8): the recieved message
    returns:
        None
    """
    global robot
    rospy.loginfo("Gripper callback") #Do nothing for now

def panTiltCallback(msg):
    """
    This function calls the camera pan/tilt routines...

    params:
        msg (std_msgs/Float32MultiArray): the recieved message
    returns:
        None
    """
    global robot
    rospy.loginfo("Pan/tilt callback") #Do nothing for now

def main():
    # reference interface globally
    global robot
    arm_config = dict(control_mode='torque')
    # start robot control
    try:
        # Home robot arm
        robot = Robot('locobot', arm_config=arm_config)
        robot.arm.go_home()

        # Initialize subscriber to respond to joystick inputs

        rospy.Subscriber(namespace + twistTopic, Twist, twistCallback)
        rospy.Subscriber(namespace + gripperStateTopic, Int8, gripperCallback)
        rospy.Subscriber(namespace + xyzArmOffsetTopic, Float32MultiArray, xyzArmCallback)
        rospy.Subscriber(namespace + panTiltOffsetTopic, Float32MultiArray, panTiltCallback)


    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
    rospy.spin()