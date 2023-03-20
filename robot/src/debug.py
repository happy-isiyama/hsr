#!/usr/bin/env python
# cooding: utf-8

#os
import time
import sys
import math

#ros
import rospy
from std_msgs.msg import String, Bool
import smach
import smach_ros

#import hsrb
from hsrb_interface import geometry
from hsrb_interface import Robot

#import my pkg
sys.path.append('/home/demulab/dspl_ws/src/hsr/robot/src')
from module import *
from function import *

def main():
    op=OpenPose()
    print(op)
if __name__ == "__main__":
    main()
