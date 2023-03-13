#!/usr/bin/env python3
import rospy
import os, sys
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import PointCloud2 as pcd2
from detectron2_ros.msg import Result
from sensor_msgs.msg import RegionOfInterest



class bounding2tf():
    def __init__(self):
        rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/rectified_points',pcd2, self.cb)
        rospy.Subscriber('/detectron2_ros/result', Result, self.result_cb)

    
    def cb(self, a):
        #rospy.loginfo("message subbed")
        pass

    def result_cb(self, res):
        print('sasasa')
        print(res.class_ids)



if __name__ == "__main__":
    rospy.init_node('saassa')
    bounding2tf()

    rospy.spin()
