#!/usr/bin/env python
import rospy
from pyrobot import Robot
import time
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int8
from tn_locobot_control.srv import homeCamera, homeCameraResponse
from tn_locobot_control.srv import homeArm, homeArmResponse

# define misc variables
namespace = "/pyRobot_web_data/"
twistTopic = "cmd_vel"
gripperStateTopic = "gripper_state"
panTiltOffsetTopic = "pan_tilt_offset"
xyzArmOffsetTopic = "xyz_arm_offet"
homeCameraServiceName = "homeCameraSrv"
homeArmServiceName = "homeArmSrv"

def homeArmService(req):
    """
    This function serves as the service callback to home the robot arm

    params:
        req (homeArm instance): the homeArm request
    returns:
        response (homeArmResponse instance): the service result
    """
    global robot

    # home the arm
    robot.arm.go_home()

    # return status
    return homeArmResponse(True)

def homeCameraService(req):
    """
    This function serves as the service callback to home the camera pan-tilt base

    params:
        req (homeCamera instance): the homeCamera request
    returns:
        response (homeCameraResponds instance): the service result
    """

    global robot

    # home the camera
    bot.camera.reset()

    # return status
    return homeCameraResponse(True)

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
    # extract message components and normalize - joystick provides [-100,100] and 
    # we will scale to [-0.1,0.1]
    arm_x = msg.data[0]/5000.0
    arm_y = msg.data[1]/5000.0
    arm_z = msg.data[2]/5000.0

    if (arm_x == 0 and arm_y == 0 and arm_z == 0):
        rospy.loginfo("no arm movement requested")
    
    else:
        # displacement = np.array([arm_x, arm_y, arm_z])
        # success = robot.arm.move_ee_xyz(displacement, plan=False)
        # rospy.loginfo("tried to move arm")
        displacement = np.array([arm_x, arm_y, arm_z])
        t,r,q = robot.arm.pose_ee
        translation = np.add(np.asarray(t).flatten(), displacement)
        orientation = np.asarray(r)
        ident = np.eye(3)
        orientation[:,2] = ident[:,2]
        orientation[2,:] = ident[2,:]
        robot.arm.set_ee_pose(translation, orientation, plan=False)
        rospy.loginfo("translation was %s", str(translation))
        rospy.loginfo("orientation was %s", str(orientation))

 
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
    # extract message components and scale (need to validate scale factors - these are wags based on teleop config)
    fwdRev = (msg.linear.x)/3
    spin = (msg.angular.z)/3

    # Reduce cross-coupling of commands
    if (spin<0.1 and fwdRev>5): spin=0
    if (fwdRev < 0.5 and spin>0.2): fwdRev=0

    # Pass command to robot base
    execution_time = 0.005   # match with web command refresh rate
     
    robot.base.set_vel(fwd_speed=fwdRev, 
                   turn_speed=spin, 
                   exe_time=execution_time)

def gripperCallback(msg):
    """
    This function calls the open and close gripper routines.

    params:
        msg (std_msgs/Int8): the recieved message
    returns:
        None
    """
    global robot
    global gripper_state
    rospy.loginfo("In gripper callback with req: %s", str(msg.data))
    if (msg.data==1 and gripper_state != 0):
        robot.gripper.open(wait=True)
        gripper_state = robot.gripper.get_gripper_state()
        rospy.loginfo("Gripper open: %s", str(gripper_state))
    elif (msg.data == 0 and (gripper_state != 3 and gripper_state != 2)):
        gripper_state = robot.gripper.close(wait=True)
        gripper_state = robot.gripper.get_gripper_state()
        rospy.loginfo("Gripper closed: %s", str(gripper_state))


def panTiltCallback(msg):
    """
    This function calls the camera pan/tilt routines...

    params:
        msg (std_msgs/Float32MultiArray): the recieved message
    returns:
        None
    """
    global robot
    cam_pose = msg.data
    if (cam_pose[0] != 0):
        pan_sign = -1 if cam_pose[0]>0 else 1
        robot.camera.set_pan(robot.camera.get_pan() + pan_sign * 0.1)
        rospy.loginfo("pan set")
    if (cam_pose[1] != 0):
        tilt_sign = -1 if cam_pose[1]>0 else 1
        robot.camera.set_tilt(robot.camera.get_tilt() + tilt_sign * 0.1)
        rospy.loginfo("tilt set")

def main():
    # reference interface globally
    global robot
    global gripper_state
    arm_config = dict(control_mode='torque')
    use_laser = True
    if (use_laser):
        modTwistTopic = "/artificial_potential/twist"
    else:
        modTwistTopic = namespace + twistTopic
    # start robot control
    try:
        # Home robot arm
        robot = Robot('locobot', arm_config=arm_config)
        robot.arm.go_home()
        gripper_state = robot.gripper.get_gripper_state()

        # initialize services
        homeCameraServ = rospy.Service(homeCameraServiceName, homeCamera, homeCameraService)
        homeArmServ = rospy.Service(homeArmServiceName, homeArm, homeArmService)

        # Initialize subscriber to respond to joystick inputs
        rospy.Subscriber(modTwistTopic, Twist, twistCallback)
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
