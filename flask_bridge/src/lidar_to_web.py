#!/usr/bin/env python

# Author: Trevor Sherrard
# Since: 06/13/2021
# Course: Summer 2021 directed research
# Purpose: This node subscribes to existing Lidar data, determines if
#          scan returns are within a given danger threshold, and sends
#          an updated copy the current state of the robot peripherals
#          to the web-backend for display within the web-interface.

import rospy
import json
import requests
import collections
import time
from WindowedAverage import MovingWindowAverage
from KalmanFilter import KalmanFilter
from sensor_msgs.msg import LaserScan
from flask_bridge.srv import setLidarThresh, setLidarThreshResponse

# define topic name
laserscan_topic = "/scan"

# define service name
setLidarThresh_name = "setLidarThreshVals"

# define constants based on web interface
NUM_DIRECTIONS = 4
NUM_INDICATORS_PER_DIRECTION = 10
interface_URL = "http://165.227.213.213:5000"
lidar_endpoint = "/setLiDARdata"

# define LiDAR constants (intrinsic to sensor)
angle_inc = 0.0124666374177
angle_min = -3.14159274101
angle_max = 3.14159274101
total_angle = angle_max - angle_min

# define list of chunk limits
chuncked_angles = []

# define list to hold last scan's thresholded state
global last_threshed
last_threshed = []

# define variables needed for rate limiting
global last_time
last_time = 0

# define thresholds for color vals
global MED_THRESH
global CLOSE_THRESH
MED_THRESH = 0.8
CLOSE_THRESH = 0.4
REFRESH_TIME = 0.25 # 250 ms

# init list of moving average objects for lidar smoothing
global MovingAverageList
MovingAverageList = []
WINDOW_SIZE = 6

# init list of Kalman filtering object for lidar smoothing
global KalmanSmoothingList
KalmanSmoothingList = []
measure_err = 0.75 # expected varience in averaged lidar data.
process_varience = 0.5 # how fast does our measurment move?

# set Smoothing Method here
USING_WINDOW = False
USING_KALMAN = True

def update_lidar_tresh_service(req):
    """
    this function serves as the ROS service to unpack data recieved 
    from the web backend through the ROS Flask bridge and 
    set global variables in this file to said values

    params;
        req (setLidarThresh instance): data sent to request
    returns:
        setLidarThreshResponse (setLidarThreshResponse instance):
                                ROS service response
    """
    # reference globals
    global MED_THRESH
    global CLOSE_THRESH

    # unpack request data
    close_thresh = req.close_tresh
    very_close_thresh = req.very_close_thresh

    # update globals
    MED_THRESH = close_thresh
    CLOSE_THRESH = very_close_thresh

    # inform system of update
    rospy.loginfo('lidar threshold values updated!')

    return setLidarThreshResponse(True)

def send_lidar_data(lidar_list):
    # jsonify list
    list_str = json.dumps(lidar_list)

    # make post req
    total_interface_URL = interface_URL + lidar_endpoint
    requests.post(total_interface_URL, json=list_str)

def proc_scan(msg):
    # reference globals
    global last_threshed
    global last_time
    global CLOSE_THRESH
    global MED_THRESH
    global MovingAverageList
    global KalmanSmoothingList

    # divide recieved scans into 40 chunks of concurrent scans
    list_index = 0
    total_count = 0
    current_angle = angle_min
    last_angle_chunk = angle_min
    current_angle_max = chuncked_angles[list_index]
    current_point_chunks = []
    total_point_chunks = []
    threshed_chunks = []
    num_scans = len(msg.ranges)

    for scan in msg.ranges:
        # if current return is within current angle window, add it to the running list
        if(current_angle > last_angle_chunk and current_angle < current_angle_max):
            current_point_chunks.append(scan)

        # otherwise...
        elif(current_angle > current_angle_max):
            # make sure to add current scan to running list
            current_point_chunks.append(scan)

            # add running list to total list of chunks
            total_point_chunks.append(current_point_chunks)

            # reset low bound for angle window
            last_angle_chunk = current_angle_max

            # set new upper bound for angle window
            list_index += 1
            current_angle_max = chuncked_angles[list_index]

            # reset running list
            current_point_chunks = []

        # if we are at the end of the scan, we need to manually append
        # point chunk to larger list
        if(total_count == num_scans - 1):
            total_point_chunks.append(current_point_chunks)

        # increment current angle
        current_angle += angle_inc
        total_count += 1

    # average each chunk and threshold the average into one of three levels
    # (close = 2, medium = 1, far = 0)
    for chunk in total_point_chunks:
        count = 0

        # compute chunk avg
        chunk_avg = sum(chunk) / len(chunk)

        # perform smoothing
        res = 0
        if(USING_WINDOW):
            # append thresholded value to moving averager
            res = MovingAverageList[count].addAndProc(chunk_avg)
        elif(USING_KALMAN):
            res = KalmanSmoothingList[count].updateEstimate(chunk_avg)

        # threshold averaged value 
        cur_val = 0
        if(res < MED_THRESH and res > CLOSE_THRESH):
            color_val = 1
            diff = MED_THRESH - CLOSE_THRESH 
            fade_value = (res - diff)/diff
            cur_val = color_val + fade_value

        elif(res < CLOSE_THRESH):
            color_val = 2
            fade_value = res/CLOSE_THRESH
            cur_val = color_val + fade_value

        threshed_chunks.append(cur_val)

        # increment count
        count += 1

    # only send data if list has changed since last scan
    if(collections.Counter(threshed_chunks) != collections.Counter(last_threshed)):
        # only send data if 500ms have elapsed since last send
        now = time.time()
        if(now - last_time > REFRESH_TIME):
            send_lidar_data(threshed_chunks)
            last_time = time.time()

    # update last threshed values list
    last_threshed = threshed_chunks

def main():
    # init globals
    global MovingAverageList
    global KalmanSmoothingList

    rospy.init_node('lidar_to_web')
    rospy.loginfo('lidar_to_web node initialized!')

    # start subscriber
    rospy.Subscriber(laserscan_topic, LaserScan, proc_scan)

    # set up lidar threshold update service
    rospy.Service(setLidarThresh_name, setLidarThresh, update_lidar_tresh_service)
    rospy.loginfo('set LiDAR threshold service up! Ready for requests...')

    # init angle limits for chuncks
    total_angle_inc = total_angle / (NUM_DIRECTIONS * NUM_INDICATORS_PER_DIRECTION)
    for i in range(1, (NUM_DIRECTIONS * NUM_INDICATORS_PER_DIRECTION) + 1):
        chuncked_angles.append(angle_min + total_angle_inc * i)

    # init moving average object list
    if(USING_WINDOW):
        for i in range(NUM_DIRECTIONS * NUM_INDICATORS_PER_DIRECTION):
            newAvg = MovingWindowAverage(WINDOW_SIZE)
            MovingAverageList.append(newAvg)

    elif(USING_KALMAN):
        for i in range(NUM_DIRECTIONS * NUM_INDICATORS_PER_DIRECTION):
            newKalman = KalmanFilter(measure_err, process_varience)
            KalmanSmoothingList.append(newKalman)

    rospy.spin()

if(__name__ == "__main__"):
    main()
