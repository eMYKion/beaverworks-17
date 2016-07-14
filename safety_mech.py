#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

OBSTACLE_SCAN_RANGE_DEGREES = 45

MINIMUM_RANGE_INDEX = int(1081 / 270 * (270 - OBSTACLE_SCAN_RANGE_DEGREES)/2)

MAXIMUM_RANGE_INDEX = int(1081 / 270 * (270 + OBSTACLE_SCAN_RANGE_DEGREES)/2)

TOTAL_OBSTACLE_SIZE_THRESHOLD=10

SEARCH_OBSTACLE_RANGE=12

#Crossing a "size" threshold over an even greater continuous search range 
#will allow the robot to stop if there are two or more obstacles in a localized range.

#If we just searched for one continuous block, then hypothetical noise for even 
#one range value within the block will cause the robot
#to crash into the block.

#launch the node
rospy.init_node('safety_mech')
rate = rospy.Rate(4)

#/safety has a higher priority than /navigation
pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=1)

def obstacle_check(msg):
	ranges = msg.ranges

	#scan through the entire front segment
	for i in range(MINIMUM_RANGE_INDEX, MAXIMUM_RANGE_INDEX+1-SEARCH_OBSTACLE_RANGE):

		#find a continuous obstacle(s)
		current_total_obstacle_size = 0

		for j in range(0, SEARCH_OBSTACLE_RANGE):

			if(ranges[i+j] < 0.51):
				current_total_obstacle_size += 1
		
		#if we see large enough obstacle(s)
		if(current_total_obstacle_size > TOTAL_OBSTACLE_SIZE_THRESHOLD):

			pub.publish(AckermannDriveStamped())#a new ackermann drive-stamp with steering_angle=0.0, speed=0.0
			

#check for obstacles every time we recieve scan information from the /scan topic
sub = rospy.Subscriber("scan", LaserScan, obstacle_check) 

#let the current node run until it's stopped
rospy.spin()

