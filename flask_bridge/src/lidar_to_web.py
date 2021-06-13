#!/usr/bin/env python

# Author: Trevor Sherrard
# Since: 06/13/2021
# Course: Summer 2021 directed research
# Purpose: This node subscribes to existing Lidar data, determines if
#          scan returns are within a given danger threshold, and sends
#          an updated copy the current state of the robot peripherals
#          to the web-backend for display within the web-interface.

import rospy
from sensor_msgs.msg import LaserScan

# define topic name
laserscan_topic = "/scan"

# define LiDAR constants (intrinsic to sensor)
angle_inc = 0.0124666374177
angle_min = -3.14159274101
angle_max = 3.14159274101

def proc_scan(msg):
    print(len(msg.ranges)*angle_inc)

def main():
    rospy.init_node('lidar_to_web')
    rospy.loginfo('lidar_to_web node initialized!')

    # start subscriber
    rospy.Subscriber(laserscan_topic, LaserScan, proc_scan)

    rospy.spin()

if(__name__ == "__main__"):
    main()
