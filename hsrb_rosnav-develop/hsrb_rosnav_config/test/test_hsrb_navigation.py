#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2018 Toyota Motor Corporation
import math
import sys
import unittest

import actionlib
import angles
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import rostest
import tf


class TestActionGoal(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_hsrb_navigation')

    def setUp(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.listener = tf.TransformListener()

    def check_goal(self, x, y, angle):
        (trans, rot) = self.listener.lookupTransform(
            'map', 'base_link', rospy.Time(0))
        rpy = tf.transformations.euler_from_quaternion(rot)

        self.assertAlmostEqual(trans[0], x, delta=0.2)
        self.assertAlmostEqual(trans[1], y, delta=0.2)
        self.assertAlmostEqual(rpy[0], 0.0, delta=0.005)
        self.assertAlmostEqual(rpy[1], 0.0, delta=0.005)

        diff = angles.normalize_angle(rpy[2]) - angles.normalize_angle(angle)
        if diff > math.pi:
            diff = diff - math.pi
        if diff < -math.pi:
            diff = diff + math.pi
        self.assertAlmostEqual(diff, 0.0, delta=0.5)

    def make_move_base_goal(self, x, y, angle, time):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = tf.transformations.quaternion_from_euler(0, 0, angle)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        return goal

    def test_move_to_goal(self):
        # Waypoint 1
        goal = self.make_move_base_goal(0.5, 0.3, math.pi, 0)
        self.client.send_goal(goal)
        result = self.client.wait_for_result(timeout=rospy.Duration(60))
        self.assertTrue(result)
        self.check_goal(0.5, 0.3, math.pi)
        # Waypoint 2
        goal = self.make_move_base_goal(0, 0, math.pi / 2, 0)
        self.client.send_goal(goal)
        result = self.client.wait_for_result(timeout=rospy.Duration(60))
        self.assertTrue(result)
        self.check_goal(0, 0, math.pi / 2)
        # Waypoint 3
        goal = self.make_move_base_goal(-0.5, 0.3, -math.pi, 0)
        self.client.send_goal(goal)
        result = self.client.wait_for_result(timeout=rospy.Duration(60))
        self.assertTrue(result)
        self.check_goal(-0.5, 0.3, -math.pi)
        # Waypoint 4
        goal = self.make_move_base_goal(0, 0, 0, 0)
        self.client.send_goal(goal)
        result = self.client.wait_for_result(timeout=rospy.Duration(60))
        self.assertTrue(result)
        self.check_goal(0, 0, 0)


if __name__ == '__main__':
    rostest.rosrun('hsrb_rosnav_config', 'test_hsrb_navigation',
                   __name__, sys.argv)
