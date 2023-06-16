#!/usr/bin/env python
import os
import numpy as np
import tensorflow as tf

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class MotionController:
    def __init__(self, weightPath):
        self.loadModel(weightPath)
        
        rospy.init_node('Controller', anonymous=True)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def loadModel(self, weightPath):
        # recreate model
        self.graph = tf.get_default_graph()
        self.model = tf.keras.Sequential([
                tf.keras.layers.Dense(64, activation='relu', input_dim=90),
                tf.keras.layers.Dense(32, activation='relu'),
                tf.keras.layers.Dense(2)
            ])
    
        self.model.load_weights(weightPath, by_name=False)
        print(self.model.summary())

    def scan_cb(self, scan):
        # take laser scan data from 45 ~ 135 degree
        scan_values = np.array(list(scan.ranges[45:-45])).reshape([1,90])
        scan_values[scan_values==np.inf] = 3.5

        with self.graph.as_default():
            res = self.model.predict(scan_values)[0]
        
        linear, angular = res[0], res[1]
        rospy.loginfo("linear speed: %s, angular speed: %s"%(linear, angular))

        vel = Twist()
        vel.linear.x = linear
        vel.angular.z = angular
        self.vel_pub.publish(vel)

if __name__=="__main__":
    try:
        filepath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), os.pardir, 'model/my_model'))

        v = MotionController(filepath)
        
        rospy.spin()
        
    except rospy.ROSInterruptException as e:
        print(e)
