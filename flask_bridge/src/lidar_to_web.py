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
from sensor_msgs.msg import LaserScan

# define topic name
laserscan_topic = "/scan"

# define constants based on web interface
NUM_DIRECTIONS = 4
NUM_INDICATORS_PER_DIRECTION = 10
interface_URL = "http://165.227.213.213:5000"
lidar_endpoint = "/getLiDARdata"

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

# define thresholds for color vals
MED_THRESH = 0.8
CLOSE_THRESH = 0.4

def send_lidar_data(lidar_list):
    # jsonify list
    list_str = json.dumps(lidar_list)

    # make post req
    total_interface_URL = interface_URL + lidar_endpoint
    requests.post(total_interface_URL, json=list_str)

def proc_scan(msg):
    # reference globals
    global last_threshed

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
        cur_val = 0

        # compute chunk avg
        chunk_avg = sum(chunk) / len(chunk)

        # check if chunk avg is within threshold
        if(chunk_avg < MED_THRESH and chunk_avg > CLOSE_THRESH):
            cur_val = 1
        elif(chunk_avg < CLOSE_THRESH):
            cur_val = 2

        # append thresholded value
        threshed_chunks.append(cur_val)

    # only send data if list has changed since last scan
    if(collections.Counter(threshed_chunks) != collections.Counter(last_threshed)):
        send_lidar_data(threshed_chunks)

    # update last threshed values list
    last_threshed = threshed_chunks

def main():
    rospy.init_node('lidar_to_web')
    rospy.loginfo('lidar_to_web node initialized!')

    # start subscriber
    rospy.Subscriber(laserscan_topic, LaserScan, proc_scan)

    # init angle limits for chuncks
    total_angle_inc = total_angle / (NUM_DIRECTIONS * NUM_INDICATORS_PER_DIRECTION)
    for i in range(1, (NUM_DIRECTIONS * NUM_INDICATORS_PER_DIRECTION) + 1):
        chuncked_angles.append(angle_min + total_angle_inc * i)

    rospy.spin()

if(__name__ == "__main__"):
    main()
