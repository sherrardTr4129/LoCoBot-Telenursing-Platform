#!/usr/bin/env python

# Author: Trevor Sherrard
# Course: Directed Research
# Since: March 7th, 2021
# Description: This file serves as the robot side of the cross-domain robot control scheme

# import required libraries
import rospy
import time
import threading
from flask import Flask, request, jsonify
from flask_cors import CORS
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int8
from tn_locobot_control.srv import homeCamera, homeCameraResponse
from tn_locobot_control.srv import homeArm, homeArmResponse

# initialize flask
app = Flask(__name__)
CORS(app)

# init globals
global cmdVelPub
global gripperStatePub
global armOffsetPub
global panTiltOffsetPub

# define misc variables
namespace = "/pyRobot_web_data/"
twistTopic = "cmd_vel"
gripperStateTopic = "gripper_state"
panTiltOffsetTopic = "pan_tilt_offset"
xyzArmOffsetRopic = "xyz_arm_offet"
homeCameraServiceName = "homeCameraSrv"
homeArmServiceName = "homeArmSrv"

@app.route('/homeCamera', methods=['GET'])
def homeCameraCB():
    """
    This function serves as a designated ROS bridge endpoint 
    that is used to home the camera to the default location

    params:
        None
    returns:
        None
    """
    # wait for service to come up
    rospy.wait_for_service(homeCameraServiceName)

    # make service call
    try:
        serv_client = rospy.ServiceProxy(homeCameraServiceName, homeCamera)
        resp = serv_client(True)
    except rospy.ServiceException as error:
        rospy.logerr("home camera service call failed: %s" % error)

@app.route('/homeArm', methods=['GET'])
def homeArmCB():
    """
    This function serves as a designated ROS bridge endpoint
    that is used to home the arm to the default configuration.

    params:
        None
    returns:
        None
    """
    # wait for service to come up
    rospy.wait_for_service(homeArmServiceName)
    
    # make service call
    try:
        serv_client = rospy.ServiceProxy(homeArmServiceName, homeArm)
        resp = serv_client(True)
    except rospy.ServiceException as error:
        rospy.logerr("home arm service call failed: %s" % error)

@app.route('/updateRobotState', methods=['POST'])
def updateRobotStateCB():
    """
    This function serves as the callback for the update robot state 
    endpoint. When an updated robot state variable is recieved here,
    it is unpacked and the components are published within the robot's
    software framework

    params:
        None
    returns:
        statusStr (str): a status String
    """
    # reference globals
    global cmdVelPub
    global gripperStatePub
    global armOffsetPub
    global panTiltOffsetPub

    # create dict from JSON
    robotObj_dict = request.get_json(force=True)

    # process dict
    fwdRev_val = float(robotObj_dict['fwdRev'])
    spin_val = float(robotObj_dict['spin'])
    tiltOffset_val = float(robotObj_dict['tiltOffset'])
    panOffset_val = float(robotObj_dict['panOffset'])
    gripperState_val = int(robotObj_dict['gripperState'])
    xArmOffset_val = float(robotObj_dict['armOffset_x'])
    yArmOffset_val = float(robotObj_dict['armOffset_y'])
    zArmOffset_val = float(robotObj_dict['armOffset_z'])

    # construct ROS messages

    # twist message
    cmdVelTwist = Twist()
    cmdVelTwist.linear.x = fwdRev_val
    cmdVelTwist.angular.z = spin_val

    # gripper state message
    gripperStateMsg = Int8()
    gripperStateMsg.data = gripperState_val

    # pan-tilt array
    panTiltList = [panOffset_val, tiltOffset_val]
    panTiltMsg = Float32MultiArray()
    panTiltMsg.data = panTiltList

    # x,y,z arm offset array
    xyzOffsetData = [xArmOffset_val, yArmOffset_val, zArmOffset_val]
    xyzOffsetMsg = Float32MultiArray()
    xyzOffsetMsg.data = xyzOffsetData

    # publish new data packets
    cmdVelPub.publish(cmdVelTwist)
    gripperStatePub.publish(gripperStateMsg)
    armOffsetPub.publish(xyzOffsetMsg)
    panTiltOffsetPub.publish(panTiltMsg)

    # return status string
    return "ACK"

def main():
    # reference globals
    global cmdVelPub
    global gripperStatePub
    global armOffsetPub
    global panTiltOffsetPub

    # init node
    rospy.init_node("robot_flask_bridge")
    rospy.loginfo("robot_flask_bridge node started!")

    # create publishers
    cmdVelPub = rospy.Publisher(namespace + twistTopic, Twist, queue_size = 1)
    gripperStatePub = rospy.Publisher(namespace + gripperStateTopic, Int8, queue_size = 1)
    armOffsetPub = rospy.Publisher(namespace + xyzArmOffsetRopic, Float32MultiArray, queue_size = 1)
    panTiltOffsetPub = rospy.Publisher(namespace + panTiltOffsetTopic, Float32MultiArray, queue_size = 1)

    # start flask as a thread
    threading.Thread(target=lambda: app.run(host="0.0.0.0", port=12345)).start()

if(__name__ == "__main__"):
    main()
