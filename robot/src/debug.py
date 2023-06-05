#!/usr/bin/env python3
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
from humanmodule import *
from function import *
from audiomodule import *
from manipulation import *

def main():
    whole_body = robot.get('whole_body')
    omni_base = robot.get('omni_base')

    whole_body.move_to_joint_positions({'head_tilt_joint':-1.0})
#    arm = Arm()
#    object_list = arm.take_list()
#    print(omni_base.pose)
#    if object_list != "no_object":
#        arm.shelf_collision(object_list,pick_object,0.02,0.05,0.3)
#        rospy.sleep(1.0)
#        try:
#            arm.instance_shelf(pick_object)
#            rospy.sleep(1.0)
#        except rospy.ROSInterruptException:
#            print("error")
#            pass
 
        #op=OpenPose()
        #print("init")
        #rospy.sleep(1)
        #a=op.human()
        #print(a)
    #arm = Arm()
    #arm.place_object()
        #ys = YesNo.voice_yseno()
if __name__ == "__main__":
    main()
