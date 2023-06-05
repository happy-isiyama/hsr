#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2018 Toyota Motor Corporation
import os
import random
import unittest

from geometry_msgs.msg import Twist
import numpy as np
from PIL import Image
import rosnode
import rospkg
import rospy
import rostest
import tf

# map上のfree画素値
_FREE = 254


def count_map_free(pgm_file):
    u"""pgmのfreeな画素の数を返す"""
    im = np.asarray(Image.open(pgm_file))
    return len(np.where(im == _FREE)[0])


class TestMapping(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_hsrb_mapping')

    def setUp(self):
        self.pub_cmd_vel = rospy.Publisher(
            '/hsrb/command_velocity', Twist, queue_size=10)
        self.listener = tf.TransformListener()
        rospy.sleep(5.0)

    def test_running(self):
        nodes = rosnode.get_node_names()
        self.assertIn('/gazebo', nodes, 'node does not exit')
        self.assertIn('/gmapping', nodes, 'node does not exit')

    def test_map(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = 1.0

        for i in range(200):
            self.pub_cmd_vel.publish(cmd_vel)
            rospy.sleep(0.1)
        # Save map image
        id_num = random.randint(1000, 10000)
        map_result = '/tmp/map_{}'.format(id_num)
        os.system("rosrun map_server map_saver -f {}".format(map_result))

        rospack = rospkg.RosPack()
        map_answer = rospack.get_path(
            'hsrb_mapping') + '/test/map_answer.pgm'
        rospy.loginfo("map_answer: {}".format(map_answer))

        # Check if the map is different
        answer = count_map_free(map_answer)
        result = count_map_free(map_result + '.pgm')
        self.assertAlmostEqual(answer, result, delta=100)


if __name__ == '__main__':
    rostest.rosrun('hsrb_mapping', 'test_hsrb_mapping', TestMapping)
