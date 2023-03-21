#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#-----------------------------------------------------------
# Title: 目的地の名前と座標を設定するサービスサーバー
# Author: 
#-----------------------------------------------------------
import rospy
import rosparam
import sys
import os
import actionlib
import time
from std_msgs.msg import String, Float64
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
sys.path.append('/home/administrator/dspl_ws/src/hsr/move_pkg/srv')
print(sys.path)
from move_pkg.srv import NaviLocation, NaviLocationResponse
from trajectory_msgs.msg import JointTrajectory
import navi_action

if __name__ == '__main__':
    rospy.init_node('navi_action', anonymous = True)
    try:
        navi_action.navigationAC('start')
        #navigationAC('start')
        navi_action.navigationAC('table_b')
        #navigationAC('table_b')
    except rospy.ROSInterruptException:
        pass
