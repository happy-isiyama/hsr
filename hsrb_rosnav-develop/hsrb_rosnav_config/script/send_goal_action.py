#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2018 Toyota Motor Corporation

import math

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
from tf import transformations

if __name__ == '__main__':

    try:
        rospy.init_node('send_goal', anonymous=True)
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.pose.position.x = -1.5
        goal.target_pose.pose.position.y = 3.0

        q = transformations.quaternion_from_euler(0.0, 0.0, math.pi / 2)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        print(goal)
        client.send_goal(goal)

        finished = client.wait_for_result()
        print("Finished : {}".format(finished))

    except rospy.ROSInterruptException:
        pass
