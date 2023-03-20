#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import time
import sys

import rospy
from std_msgs.msg import String
import smach
import smach_ros

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
        return "to_task2A"

class Task2A(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['to_taskB'])

    def execute(self,userdata):
        rospy.loginfo("----------------Task 2A--------------------")
        return "to_taskB"

class TaskB(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['finish'])

    def execute(self,userdata):
        rospy.loginfo("----------------Task B--------------------")
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
