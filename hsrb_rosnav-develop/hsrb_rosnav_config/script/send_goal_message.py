#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2018 Toyota Motor Corporation

from geometry_msgs.msg import PoseStamped
import rospy

if __name__ == '__main__':

    rospy.init_node('send_goal_message', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    # wait for /clock for simulation
    rospy.sleep(1)

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.position.x = -1.5
    msg.pose.position.y = 3.0
    msg.pose.orientation.w = 1.0

    print(msg)
    pub.publish(msg)
