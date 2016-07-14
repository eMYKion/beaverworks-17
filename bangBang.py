#!/usr/bin/env python
import rospy,  math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class wallF:
    
    def __init__(self,  threshold,  steer_angle, d_des, speed):
        self.threshold = threshold # define at what point to stop steering
        self.steer_angle = steer_angle # define how sharp the correction turns are
        self.error = 0 # initializes the running error counter
        self.d_des = d_des # takes desired dist from wall
        self.speed = speed
        self.scan = [0 for x in range(0, 10)]
        rospy.init_node("wall_bang",  anonymous=False)
        rate = rospy.Rate(10)
        
        #init the pub/subs
        self.drive = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation",  AckermannDriveStamped,  queue_size=5)
        rospy.Subscriber("scan",  LaserScan,  self.callback)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.speed = self.speed
        
        dist_trav = 5.0 
        total = 0.0
        
        time = dist_trav/self.speed
        ticks = int(time * 10)
        for t in range(ticks):
            total = 0
            for i in self.scan:
                total += i
            meanD = total/len(self.scan)
            self.error = self.d_des - meanD
            if abs(self.error) <= self.threshold: # if perf
                drive_msg.drive.steering_angle=0.0 # go straight
            elif self.error > self.threshold: # if too far right
                drive_msg.drive.steering_angle = self.steer_angle # turn left
            else: # if too far left
                steer_angle = self.steer_angle * -1 
                drive_msg.drive.steering_angle = steer_angle # turn right
            self.drive.publish(drive_msg)
            rate.sleep()
        self.drive.pub(AckermannDriveStamped())
    def callback(self,msg):
	for i in range(895, 906):
        	self.scan.append(msg.ranges[i])
    
    
if __name__ == "__main__":
    wallF(0.4,0.2,0.5,1.5)
