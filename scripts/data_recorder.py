#!/usr/bin/env python
import os
import rospy
import pandas as pd
import csv
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Recorder:
    def __init__(self, csvwriter):
        rospy.init_node('Recorder', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.vel_cb)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.scans = [0,0,0]
        self.isStart = False
        self.csvwriter = csvwriter
    
    def scan_cb(self, scan):
        # take laser scan data from 45 ~ 135 degree
        self.scans = list(scan.ranges[45:-45])

    def vel_cb(self, vel):
        self.isStart = True
        self.linear_vel = vel.linear.x
        self.angular_vel = vel.angular.z

    def write_data(self):
        if self.isStart:
            self.csvwriter.writerow(self.scans+[self.linear_vel, self.angular_vel])
            #self.data.append(self.scans+[self.linear_vel, self.angular_vel])

if __name__=="__main__":
    try:
        filepath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), os.pardir, 'data/record.csv'))
        csvfile = open(filepath, "w")
        csvwriter = csv.writer(csvfile)

        rec = Recorder(csvwriter)
        
        rate = rospy.Rate(50)
        count = 0
        while not rospy.is_shutdown():
            rec.write_data()
            rate.sleep()

            if count > 1e4:
                break
        
    except rospy.ROSInterruptException as e:
        print(e)
