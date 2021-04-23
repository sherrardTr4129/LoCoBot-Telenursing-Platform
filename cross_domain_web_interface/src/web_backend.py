#!/usr/bin/env python

# Author: Trevor Sherrard
# Since: March 6th, 2021
# Project: PyRobot Teleoperation

# import required libraries
import time
import threading
import copy
import requests
import argparse
from json import dumps
from flask import Flask, request, jsonify
from flask_cors import CORS
from FrontEndStateClass import FrontEndState

# init flask app
app = Flask(__name__)
CORS(app)

# setup global state manager
global FE_State
global prev_FE_State
global RobotURL
FE_State = FrontEndState()
prev_FE_State = FrontEndState()

# declare URL of Robot
robotEndpoint = "/updateRobotState"
homeCameraEndpoint = "/homeCamera"
homeArmEndpoint = "/homeArm"
robotHostname = ""
RobotURL = ""

# declare misc. variables
DELAY = 0.3
MAX_ZERO_COUNT = 3

@app.route('/setFwdRev', methods=['POST'])
def setFwdRev():
    """
    This function serves as the API endpoint to which updates to the fwdRev
    global value are POSTed from the frontend javascript. This function extracts the 
    payload and updates it at a global scope.

    params:
        None
    returns:
        status (String)
    """

    global FE_State
    
    # extract data
    fwdRevDict = request.get_json()
    fwdRevData = float(fwdRevDict['fwdRev'])

    # scale data
    fwdRevData = fwdRevData / 150

    # update the data in FE_State
    FE_State.set_fwd_rev(fwdRevData)
    
    # return status
    return "OK"

@app.route('/setSpin', methods=['POST'])
def setSpin():
    """
    This function serves as the API endpoint to which updates to the spin
    global value are POSTed from the frontend javascript. This function extracts the
    payload and updates it at a global scope.

    params:
        None
    returns:
        status (String)
    """
    global FE_State

    # extract data
    spinDataDict = request.get_json()
    spinData = float(spinDataDict['spin'])

    # scale data
    spinData = -spinData / 150

    # update the data in FE_State
    FE_State.set_spin(spinData)

    # return status
    return "OK"

@app.route('/setArmOffsetX', methods=['POST'])
def setArmOffsetX():
    """
    This function serves as the API endpoint to which updates to the arm_offet_x
    global value are POSTed from the frontend javascript. This function extracts the
    payload and updates it at a global scope.

    params:
        None
    returns:
        status (String)
    """
    global FE_State

    # extract data
    XArmDict = request.get_json()
    xData = float(XArmDict['xArmOffset'])

    # update the data in FE_State
    FE_State.set_arm_offset_x(xData)

    # return status
    return "OK"

@app.route('/setArmOffsetY', methods=['POST'])
def setArmOffsetY():
    """
    This function serves as the API endpoint to which updates to the arm_offet_y
    global value are POSTed from the frontend javascript. This function extracts the
    payload and updates it at a global scope.

    params:
        None
    returns:
        status (String)
    """
    global FE_State

    # extract data
    YArmDict = request.get_json()
    yData = float(YArmDict['yArmOffset'])

    # update the data in FE_State
    FE_State.set_arm_offset_y(yData)

    # return status
    return "OK"

@app.route('/setArmOffsetZ', methods=['POST'])
def setArmOffsetZ():
    """
    This function serves as the API endpoint to which updates to the arm_offet_z
    global value are POSTed from the frontend javascript. This function extracts the
    payload and updates it at a global scope.

    params:
        None
    returns:
        status (String)
    """
    global FE_State

    # extract data
    ZArmDict = request.get_json()
    zData = float(ZArmDict['zArmOffset'])

    # update the data in FE_State
    FE_State.set_arm_offset_z(zData)

    # return status
    return "OK"

@app.route('/setPanOffset', methods=['POST'])
def setPanOffset():
    """
    This function serves as the API endpoint to which updates to the pan_offset
    global value are POSTed from the frontend javascript. This function extracts the
    payload and updates it at a global scope.

    params:
        None
    returns:
        status (String)
    """
    global FE_State

    # extract data
    panOffsetDict = request.get_json()
    panOffsetData = float(panOffsetDict['panOffsetData'])

    # update the data in FE_State
    FE_State.set_pan_offset(panOffsetData)

    # return Status
    return "OK"

@app.route('/setTiltOffset', methods=['POST'])
def setTiltOffset():
    """
    This function serves as the API endpoint to which updates to the tilt_offset
    global value are POSTed from the frontend javascript. This function extracts the
    payload and updates it at a global scope.

    params:
        None
    returns:
        status (String)
    """
    global FE_State

    # extract data
    tiltOffsetDict = request.get_json()
    tiltOffsetData = float(tiltOffsetDict['tiltOffsetData'])

    # update the data in FE_State
    FE_State.set_tilt_offset(tiltOffsetData)

    # return Status
    return "OK"

@app.route('/setGripperState', methods=['POST'])
def setGripperState():
    """
    This function serves as the API endpoint to which updates to the gripper_state
    global value are POSTed from the frontend javascript. This function extracts the
    payload and updates it at a global scope.

    params:
        None
    returns:
        status (String)
    """
    global FE_State

    # extract data
    gripperStateDict = request.get_json()
    gripperStateData = int(gripperStateDict['gripperStateData'])

    # update the data in FE_State
    FE_State.set_gripper_state(gripperStateData)

    # return status
    return "OK"

def jsonifyObj(FEObj):
    """
    This function returns the JSON string representation
    of a front-end state object. 

    params:
        FEObj (Front-End State Object): The front-end state object to JSONify
    returns:
        JSONStr (str): The JSON String representation of FEObj
    """

    objDict = dict(FEObj)
    JSONStr = dumps(objDict)
    return JSONStr

def makePostReq(RobotURL, FEObj):
    """
    This function will send a JSON representation of the
    front-end state to the Robot when called.

    params:
        RobotURL (str): The URL of the robot flask app
        FEObj (Front-end state object): The FE State object to send to the robot
    returns:
        status (bool): a boolean indicating an ACK from the robot
    """

    # get JSON string
    JSONStr = jsonifyObj(FEObj)
    print(JSONStr)
    print(RobotURL)

    # make request
    statusString = requests.post(RobotURL, data=JSONStr)

    if(statusString.text == "ACK"):
        return True
    else:
        return False

@app.route('/homeArm', methods=['GET'])
def homeArmCallback():
    """
    This function serves as the backend endpoint for the home the arm
    button within the front-end interface.

    params:
        None
    returns:
        None
    """
    # make url
    homeArmURL = robotHostname + homeArmEndpoint

    # make GET request to robot
    statusString = requests.get(homeArmURL)

@app.route('/homeCamera', methods=['GET'])
def homeCameraCallback():
    """
    This function serves as the backend endpoint for the home the camera
    buttont within the front-end interface.

    params:
        None
    returns:
        None
    """
    # make url
    homeCameraURL = robotHostname + homeCameraEndpoint

    # make GET request to robot
    statusString = requests.get(homeCameraURL)

def allZero(FEObj):
    """
    This function checks the state of a given FrontEndState Object
    and determines if all class elements are equal to zero or not. 

    params:
        FEObject (FrontEndState Obj): The object to test
    returns:
        statusBool (Boolean): A boolean indicating the state of the object
    """
    objDict = dict(FEObj)
    statusBool = True

    # loop through all elements and see if 
    # any class memebers are non-zero
    for elem in objDict:
        # skip gripper state 
        if(elem == "gripperState"):
            continue
        else:
            val = objDict[elem]
            if(val != 0.0):
                statusBool = False

    return statusBool

def postIfChanged():
    """
    This function will make a POST request to the robot with an updated FE_state
    object JSON representaion when new user input is recieved, or when data axes
    are non-zero (except in the case of the push buttons). 

    params:
        None
    returns:
        None
    """
    global FE_State
    global prev_FE_State
    allZeroCount = 0

    while(True):
        # check if all class elements are zero
        allZeroBool = allZero(FE_State)
        
        # check if state has changed or values are non-zero
        if(not (FE_State == prev_FE_State) or not allZeroBool):
            # reset zero count
            allZeroCount = 0

            # make request
            makePostReq(RobotURL, FE_State)

        # if we have just transitioned back to default control state (i.e hands off)
        # send at least three data packets to make sure robot gets stop commands
        elif(FE_State == prev_FE_State and allZeroCount < MAX_ZERO_COUNT):
            # increament zero count
            allZeroCount += 1

            # make request
            makePostReq(RobotURL, FE_State)

        # otherwise, do nothing
        else:
            pass

        # update prev State
        prev_FE_State = copy.deepcopy(FE_State)

        # sleep for DELAY seconds
        time.sleep(DELAY)

def main():
    # reference globals
    global RobotURL
    # set up arg parser
    parser = argparse.ArgumentParser(description='Enter ngrok domain name for robot.')
    parser.add_argument('-u','--url',action='store',dest='url',default=None,help='<Required> url link',required=True)

    # try to get args
    result = parser.parse_args()
    robotHostname = result.url

    # construct new robot url
    RobotURL = robotHostname + robotEndpoint

    # start flask app as a thread
    threading.Thread(target=lambda: app.run(host="robotcontrol.live", port=5000)).start()

    # start the post thread
    thread = threading.Thread(target=postIfChanged)
    thread.start()

if(__name__ == "__main__"):
    main()
