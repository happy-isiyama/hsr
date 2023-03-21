#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import time
import sys

import rospy
from std_msgs.msg import String
import smach
import smach_ros

import hsrb_interface
from hsrb_interface import Robot

#sys.path.insert(0, '/home/demulab/despl_ws/src/robot/src')
#from module import *
#from function import *

robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')

class EnterRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['to_taskA'])

    def execute(self, userdata):
        rospy.loginfo("------------Enter Room-------------------")
        #tts.say('Start tidy up')
        #tts.say('please open the door')
        return "to_taskA"

class TaskA(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['to_task2A'])

    def execute(self,userdata):
        rospy.loginfo("----------------Task A--------------------")
        omni_base.go_abs(2.380020627581534, 1.056500281377312, 0.02266108632762348, 300.0) #Table_b
        omni_base.go_abs(1.595966813767852, 1.2583708350487552, -3.120435860989875, 300.0) #Table 1
        return "to_task2A"

class Task2A(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['to_taskB'])

    def execute(self,userdata):
        rospy.loginfo("----------------Task 2A--------------------")
        omni_base.go_abs(2.439885828485936, 0.12257944264739871, 0.0452156824171235, 300.0) #waypoint1
        omni_base.go_abs(3.9091511519156223, 0.15497955104277086, 0.038638968574600786, 300.0) #waypoint2
        return "to_taskB"

class TaskB(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['finish'])

    def execute(self,userdata):
        rospy.loginfo("----------------Task B--------------------")
        omni_base.go_abs(4.4331769989766485, 1.0486846497559235, 0.03786785857847326, 300.0) #shelf
        omni_base.go_abs(4.63224620012862, 1.4850550571247705, 1.609544631220165, 300.0) #human_center
        omni_base.go_abs(4.022050393438807, 2.2987666211105346, 1.6000028501606836, 300.0) #human_left
        omni_base.go_abs(5.053066552752714, 2.347473880985127, 1.5918783317735565, 300.0) #human_right
        return "finish"


def main():
    sm_tidy_up = smach.StateMachine(outcomes=['FINISH'])

    with sm_tidy_up:
        #EnterRoom
        smach.StateMachine.add('ENTER',EnterRoom(),
                transitions={'to_taskA':'TASK_A'})

        #Task A
        smach.StateMachine.add('TASK_A',TaskA(),
                transitions={'to_task2A':'TASK_2A'})

        #Task 2A
        smach.StateMachine.add('TASK_2A',Task2A(),
                transitions={'to_taskB':'TASK_B'})

        #Task B 
        smach.StateMachine.add('TASK_B',TaskB(),
                transitions={'finish':'FINISH'})

        outcome = sm_tidy_up.execute()


if __name__ =='__main__':
    #rospy.init_node('sm_tidy_up')
    main()
