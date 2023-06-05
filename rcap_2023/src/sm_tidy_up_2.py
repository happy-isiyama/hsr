#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import time
import sys

import rospy
from std_msgs.msg import String
import smach
import smach_ros

import hsrb_interface
from hsrb_interface import Robot
from hsrb_interface import geometry

sys.path.append("/home/demulab/dspl_ws/src/hsr/robot/src")
#from module import *
from sensor_msgs.msg import LaserScan
from manipulation import *
from humanmodule import *

robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')
arm = Arm()

#------------------------------------------
pick_object = "strawberries"
#------------------------------------------

shape_list = ["soccer_ball","soft_ball","base_ball",
              "tennis_ball","racquet_ball","golf_ball","stacking_blocks","brick","dice","rope","chain"]
tools_list =["pen","padlock","clamp","plastic_bolt"]
food_list = ["cracker_box","box_of_sugar","chocolate_pudding",
             "gelatin_box","meat_can","coffee",
             "tuna_can","chips","mustard ",
             "soup_can","banana","strawberries","apple",
             "lemon","peach","pear","orange","plum"]

kitchen_list = ["glass_cleaner","cleanser","sponge",
                "pitcher_base","pitcher_lid","metal_bowl","spatula",
                "wine_glass","mug"]
orientatio_based_list = ["fork","spoon","knife","scissors"]
task_items = ["lego","wood_blocks","9-hole","airplane","magazine","t-shirt","timer"]

#locatin_list 辞書型で定義
location_list ={'entrance':(0.7593438613930832, -0.06262084319702434, 0.04509086597375952),
                'enter':(1.2665689822656732, -0.03874379822393036, -1.7258606588536878),
                'drawer_right':(1.5576360589486657, -0.08550827985493153, -1.710737975204823),
                'drawer_left':(2.0091599143348717, -0.38741638952841073, -1.7048507692120505),
                'containers_A':(2.252136128072331, -0.4537567645813348, -1.72185399079363),
                'Trays_A':(2.5121977147517898, -0.47434614276926423, -1.7225263036446665),
                'Trays_B':(2.879051476095184, -0.48258444644527826, -1.54868145211231),
                'bin_A':(3.3934779358575886, -0.5544480852367322, -1.5619447096514851),
                'bin_B':(3.960155677581472, -0.5291122666830548, -1.6222167495189816),
                'pick_A':(0.9812442860658123, 0.4591283889671645, 1.5331071660493234),
                'pick_B':(1.671502283990401, 0.5034319187174363, 1.4219231901676603),
                'pick_C':(2.5305265301234625, 0.5296583412704511, 1.463515796810405),
                'table_B':(2.1037579497598538, 1.2794696599556121, 1.5249952219681961),
                'waypoint_1':(3.698556817104813, 0.1579836268110597, 1.6062332436121731),
                'task2a_start':(3.700136780324349, 1.9313400273376218, 1.5213156770688085),
                'task2a_end':(3.6629467653550996, 3.692564971029578, 1.4208711519579684),
                'shelf_right':(3.2595201101875912, 3.982557296669328, 1.552463855003606),
                'shelf_center':(3.0643157685051796, 3.9576666351181777, 1.576485633243085),
                'shelf_left':(2.801047491189583, 3.9277634025740573, 1.5072258200410695),
                'human_center':(2.3467239615914743, 3.5442556901153144, 3.113921765258759),
                'human_left':(1.583878891773181, 3.0747168094238257, 3.0842118719946323),
                'human_right':(1.6380188914519145, 3.9927658472039997, 3.133774307508016)}
                

def main():
    rospy.loginfo("----------------Task B--------------------")
    omni_base.go_abs(location_list['waypoint_1'][0],location_list['waypoint_1'][1],location_list['waypoint_1'][2])
    rospy.sleep(5.0)
    whole_body.move_to_go()
    rospy.sleep(1.0)

    omni_base.go_abs(location_list['shelf_center'][0],location_list['shelf_center'][1],location_list['shelf_center'][2])
    rospy.sleep(2.0)
    whole_body.move_to_go()
    rospy.sleep(1.0)

    shelf_heights = [0.51, 0.795, 1.06]
    found_object = False
    for height in shelf_heights:
        whole_body.move_to_joint_positions({'arm_lift_joint': height})
        rospy.sleep(1.0)
        take_list = arm.take_object()
        if take_list != "no_object":
            found_object = True
            break

    if not found_object:
        rospy.loginfo("No object found on any shelf.")
        return "finish"

    arm.instance_shelf('object')
    omni_base.go_abs(location_list['chair_center'][0],location_list['chair_center'][1],location_list['chair_center'][2])

    rospy.sleep(2.0)
    result = "false"
    op = OpenPose()
    while result != "left" and result != "right":
        tts.say('please raise your hands')
        rospy.sleep(2.0)
        op = OpenPose()
        result = op.human()
        rospy.sleep(1.0)
    if result == "left":
        omni_base.go_abs(location_list['chair_left'][0],location_list['chair_left'][1],location_list['chair_left'][2])
        rospy.sleep(2.0)
        arm.place_object()
        gripper.command(1.0)
        rospy.sleep(2.0)
    elif result == "right":
        omni_base.go_abs(location_list['chair_right'][0],location_list['chair_right'][1],location_list['chair_right'][2])
        rospy.sleep(2.0)
        arm.place_object2()
        rospy.sleep(2.0)
        gripper.command(1.0)
        rospy.sleep(2.0)

    return "finish"


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
