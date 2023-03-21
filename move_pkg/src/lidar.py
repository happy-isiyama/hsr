#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
import rospy
import controller_manager_msgs.srv #twist型
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class Door():
    def __init__(self):
        rospy.init_node('lidar', anonymous=True)
        self.odom_sub = rospy.Subscriber('/hsrb/base_scan', LaserScan, self.lidar_callback)
        self.vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)
        self.front_laser_dist = 999.9
        
        while pub.get_num_connections() == 0:
        rospy.sleep(0.1)

        # make sure the controller is running
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers',controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
        rospy.sleep(0.1)
        for c in list_controllers().controller:
            if c.name == 'omni_base_controller' and c.state == 'running':
                running = True

    def lidar_callback(self, msg):
        self.front_laser_dist = receive_msg.ranges[359]

    def main(self):
        steps = 0
        
        while
            if:
                tw = geometry_msgs.msg.Twist()
                tw.linear.x = 1.0


if __name__ == '__main__':
    try:
        robot = Door()
        robot.main()
    except rospy.ROSInterruptException: pass
