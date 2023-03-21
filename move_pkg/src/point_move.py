#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#-----------------------------------------------------------
# Title: 目的地の名前と座標を設定するサービスサーバー
# Author: 
#-----------------------------------------------------------
import controller_manager_msgs.srv
import rospy
import trajectory_msgs.msg
import math
import actionlib
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf.transformations


class Point():

    #Unity-book_p.306    
    def goal_pose(self, pose):

        goal_pose = MoveBaseGoal()
        goal_pose.target_pose.header.frame_id = 'map'

        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]

        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

        

def main():

    print("0")

    rospy.init_node("goal_pose")

    client = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

    client.wait_for_server()

    def shutdown():
        client.cancel_goal()

    rospy.on_shutdown(shutdown)

    state = 0


    #座標が怪しい
    while(1):
        if state == 0:
            state = 1
            print("start")
            pose = [(0.0005722045898437491, 0.010295867919921873, -0.0009202081653423427), (0.0, 0.0, 0.4794253528, 0.8775826634)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(1.0)

        elif state == 1:
            state = 2
            print("waypoint_1")
            pose = [(2.5093498229980464, -0.013275146484375017, -0.00526607793232961), (0.0, 0.0, 0.4794194544, 0.8775858857)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(3.0)

        elif state == 2:
            print("table_b")
            state = 3
            pose = [(2.4376296989397743, 0.9172363281380347, -0.01275949014745991), (0.0, 0.0, 0.4793898181, 0.8776020751)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(3.0)

        elif state == 3:
            print("table 1")
            state = 4
            pose = [(1.631744384765625, 1.178985595703125, 0.9999772007029489), (0.0, 0.0, 0.0033763109, 0.9999943002)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(3.0)
            
        elif state == 4:
            print("table 2")
            state = 5
            pose = [(1.6449890136718746, 1.7370719909667964, 0.9997430495554256), (0.0, 0.0, 0.0113337203, 0.9999357713)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(3.0)
            
        elif state == 5:
            print("table 3")
            state = 6
            pose = [(1.6542434692382815, 2.271228790283203, 0.9995841760753131), (0.0, 0.0, 0.0144171541, 0.9998960674)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(3.0)
            
        elif state == 6:
            print("waypoint_1 2")
            state = 7
            pose = [(2.5093498229980464, -0.013275146484375017, -0.00526607793232961), (0.0, 0.0, 0.4794194544, 0.8775858857)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(3.0)
            
        elif state == 7:
            print("waypoint_2")
            state = 8
            pose = [(4.033771514892578, -0.0799789428710938, 0.014953091958708475), (0.0, 0.0, 0.4793764794, 0.8776093613)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(3.0)
            
        elif state == 8:
            print("shelf")
            state = 9
            pose = [(4.523284912109375, 0.7895126342773439, 0.027396467809239593), (0.0, 0.0, 0.4792608282, 0.8776725235)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(3.0)
            
        elif state == 9:
            print("human_center")
            state = 10
            pose = [(4.733039855957032, 1.214088439941406, 0.6849129776025903), (0.0, 0.0, 0.3563069432, 0.9343689647)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(3.0)
            
        elif state == 10:
            print("human_right")
            state = 11
            pose = [(5.175518035888674, 2.096611022949219, 0.6950105626940589), (0.0, 0.0, 0.3518060188, 0.9360729273)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(3.0)        

        elif state == 11:
            print("human_left")
            state = 12
            pose = [(4.150890350341797, 2.096656799316406, 0.6967783121255604), (0.0, 0.0, 0.3510041512, 0.9363739028)]
            mv = Move()
            goal = mv.goal_pose(pose)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10))
            rospy.sleep(3.0)        
        
        elif state == 13:
            print("end")
            break

if __name__ == '__main__':
    main()

