#!/usr/bin/env python3
# Copyright (C) 2016 Toyota Motor Corporation
#import actionlib
#from actionlib_msgs.msg import GoalStatus
#from geometry_msgs.msg import Point, PoseStamped, Quaternion
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
#from std_srvs.srv import Empty
import rospy
import numpy
import sys
import math
import dooropen import *
#import tf.transformations
sys.path.append("/home/demulab/dspl_ws/src/hsr/robot/src") 
import hsrb_interface
from hsrb_interface import Robot

robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')

class Nav():
   
    def point1(self):

        result = main()
        if result == "True"
            omni_base.go_abs(0.1, 0.0, 0.0)
            print("finish")
        else:
            print("stop")
        
def main():    
    nav = Nav()
    nav.point1()

if __name__ =='__main__':
    main()
