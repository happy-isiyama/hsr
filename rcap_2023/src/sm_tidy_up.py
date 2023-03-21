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
        omni_base.go_abs(2.4435990175, 0.2461424994, 0.6298248369, 300.0) #Table_b
        omni_base.go_abs(2.4793845126, -0.6980658831, 1.5002928018, 300.0) #Table 1
        return "to_task2A"

class Task2A(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['to_taskB'])

    def execute(self,userdata):
        rospy.loginfo("----------------Task 2A--------------------")
        omni_base.go_abs(2.3831604950, -0.0000165905, -0.0105738366, 300.0) #waypoint1
        omni_base.go_abs(2.6559186638, -0.0162398292, -0.0356263708, 300.0) #waypoint2
        return "to_taskB"

class TaskB(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['finish'])

    def execute(self,userdata):
        rospy.loginfo("----------------Task B--------------------")
        omni_base.go_abs(2.7167162771, 0.0602892998, 0.3325988124, 300.0) #shelf
        omni_base.go_abs(2.7835146149, -0.1906639474, 0.5368062753, 300.0) #human_center
        if human == left: #仮の条件分岐
            omni_base.go_abs(2.7351899836, -0.1230322638, 0.9608355728, 300.0) #human_left
        else:
            omni_base.go_abs(2.8110376842, -0.1302287473, 0.7915451677, 300.0) #human_right
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
    rospy.init_node('sm_tidy_up')
    main()
