#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#-----------------------------------------------------------
# Title: 目的地の名前と座標を設定するサービスサーバー
# Author: 
#-----------------------------------------------------------
#import controller_manager_msgs.srv
import rospy
#import trajectory_msgs.msg
#import math
#import actionlib
import tf
#from actionlib_msgs.msg import GoalStatus
#from geometry_msgs.msg import Point, PoseStamped, Quaternion
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
#import tf.transformations
import hsrb_interface
from hsrb_interface import Robot

robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')

class Point():
    
    def start():
        omni_base.go_abs(0.0011255792, 0.0205920460, -0.0018288272, 300.0)
        
    def table_b():
        omni_base.go_abs(2.4435990175, 0.2461424994, 0.6298248369, 300.0)
        
    def table1():
        omni_base.go_abs(2.4793845126, -0.6980658831, 1.5002928018, 300.0)
        
    def table2():
        omni_base.go_abs(2.4973775538, -0.4977993393, 1.7944799565, 300.0)

    def table3():
        omni_base.go_abs(2.5496542044, -0.3651279915, 1.9950667613, 300.0)
        
    def table_a():
        omni_base.go_abs(2.7739978344, 0.3877939076, 1.6305149937, 300.0)
        
    def waypoint_1():
        omni_base.go_abs(2.3831604950, -0.0000165905, -0.0105738366, 300.0)    
        
    def waypoint_2():
        omni_base.go_abs(2.6559186638, -0.0162398292, -0.0356263708, 300.0)

    def shelf():
        omni_base.go_abs(2.7167162771, 0.0602892998, 0.3325988124, 300.0)
        
    def human_center():
        omni_base.go_abs(2.7835146149, -0.1906639474, 0.5368062753, 300.0)

    def human_left():
        omni_base.go_abs(2.7351899836, -0.1230322638, 0.9608355728, 300.0)
        
    def human_right():
        omni_base.go_abs(2.8110376842, -0.1302287473, 0.7915451677, 300.0)
        
if __name__ == '__main__':
    try:
        Point()
        Point.start()
        Point.waypoint_1()
        Point.table_b()
        Point.table1()
        Point.table2()
        Point.table3()
        Point.table_a()
        Point.waypoint_1()
        Point.waypoint_2()
        Point.shelf()
        Point.human_center()
        Point.human_left()
        Point.human_right()
    except rospy.ROSInterruptException:
        pass

