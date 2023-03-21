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
sys.path.append('/home/kohei/dspl_ws/src/hsr/move_pkg/srv')
print(sys.path)
from move_pkg.srv import NaviLocation, NaviLocationResponse
from trajectory_msgs.msg import JointTrajectory


class Move():
    """
    def __init__(self):
        #self.pub = rospy.Publisher('/hsrb/omni_base_controller/command', JointTrajectory, queue_size = 10)
        #self.traj_value = JointTrajectory()
        self.ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def x_move():
        omni_base.go_rel(1.0, 0.0, 0.0, 100.0)
        
        # wait to establish connection between the controller
        while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
        if c.name == 'omni_base_controller' and c.state == 'running':
            running = True
        #traj_value.linear.x = 1.0
    
    def x1_move():
        omni_base.go_rel(-1.0, 0.0, 0.0, 100.0)
        #traj_value.linear.x = -1.0
        
    def y_move():
        omni_base.go_rel(0.0, 1.0, 0.0, 100.0)
        #traj_value.linear.y = 1.0

    def y_move():
        omni_base.go_rel(0.0, -1.0, 0.0, 100.0)
        #traj_value.linear.y = -1.0
    """

    def navigationAC(target_name):
        location_dict = rosparam.get_param('/location')
        if target_name in location_dict:
            coord_list = location_dict[target_name]
            rospy.loginfo("Start Navigation")

            ac.wait_for_server()
            clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = coord_list[0]
            goal.target_pose.pose.position.y = coord_list[1]
            goal.target_pose.pose.orientation.z = coord_list[2]
            goal.target_pose.pose.orientation.w = coord_list[3]

            clear_costmaps()
            rospy.wait_for_service('move_base/clear_costmaps')
            rospy.sleep(0.3)
            ac.send_goal(goal)
            state = ac.get_state()
            count = 0 # clear_costmapsの実行回数をカウンタ
            clear = 0 
            while not rospy.is_shutdown():
                state = ac.get_state()
                if state == 1:
                    rospy.loginfo('Running...')
                    rospy.sleep(1.0)
                    clear += 1
                elif state == 3:
                    rospy.loginfo('Navigation success!!')
                    state = 0
                    return True
                elif state == 4:
                    if count == 3:
                        count = 0
                        rospy.loginfo('Navigation Failed')
                        return False
                    else:
                        rospy.loginfo('Clearing Costmaps')
                        clear_costmaps()
                        ac.send_goal(goal)
                        rospy.loginfo('Send Goal')
                    count += 1

                print(clear)
                if clear == 15:
                    clear_costmaps()
                    clear = 0

        else:
            return False

if __name__ == '__main__':
    rospy.init_node('navi_action', anonymous = True)
    try:
        navigationAC('start')
        navigationAC('table_b')
        navigationAC('table 1')
        navigationAC('table 2')
        navigationAC('table 3')
        navigationAC('table_a')
        navigationAC('waypoint_1')
        navigationAC('waypoint_2')
        navigationAC('shelf')
        navigationAC('human_center')
        navigationAC('human_left')
        navigationAC('human_right')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
