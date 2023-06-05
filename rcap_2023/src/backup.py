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
pick_object = "lemon"
#------------------------------------------

shape_list = ["soccer_ball","soft_ball","base_ball",
              "tennis_ball","racquet_ball","golf_ball","stacking_blocks","brick","dice","rope","chain"]
tool_list =["pen","padlock","clamp","plastic_bolt"]
food_list = ["cracker_box","box_of_sugar","chocolate_pudding",
             "gelatin_box","meat_can","coffee",
             "tuna_can","chips","mustard ",
             "soup_can","banana","strawberry","apple",
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
                'containers_B':(2.352136128072331, -0.4537567645813348, -1.72185399079363),
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


class EnterRoom(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['to_taskA'])
        rospy.Subscriber("/hsrb/base_scan",LaserScan,self.scan_callback)
        self.range =0.0
 
    def scan_callback(self,msg):
        self.range = msg.ranges 


    def execute(self, userdata):
        rospy.loginfo("------------Enter Room-------------------")
        tts.say('start tidy up')
        rospy.sleep(1.0)
        tts.say('please open the door')
        rospy.sleep(1.0)
        #self.rangeが0.5より小さいときはループ
        while self.range[165] < 0.5:
            tts.say('please open the door')
            rospy.sleep(1.0)
        tts.say('thank you')
        rospy.sleep(1.0)
        #1m前進
        omni_base.go_rel(1.0,0.0,0.0, 300.0)
        rospy.sleep(5.0)

        return "to_taskA"

class TaskA(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['to_taskB'])

    def execute(self, userdata):
        rospy.loginfo("----------------Task A--------------------")

        omni_base.go_abs(location_list['pick_A'][0],location_list['pick_A'][1],location_list['pick_A'][2])
        rospy.sleep(5.0)

        while rospy.is_shutdown() == False:
            whole_body.move_to_go()
            #首を下に向ける
            whole_body.move_to_joint_positions({'head_tilt_joint': -0.8})
            rospy.sleep(3.0)
            #手を閉じる
            rospy.sleep(3.0)
            object_list = arm.take_list()
            #手を開ける
            rospy.sleep(3.0)
            gripper.command(1.0)
            rospy.sleep(3.0)
            if object_list == "no_object":
                print("no object")
                break
            else:
                #object_listの中身を発話
                print(object_list[0][0])
            
                try:
                    arm.instance_floor(object_list[0][0])
                    rospy.sleep(3.0)
                except:
                    print("error")
                    pass
                whole_body.move_to_go()
                rospy.sleep(3.0)
                
                if object_list[0][0] in shape_list:
                    rospy.sleep(2.0)
                    omni_base.go_abs(location_list['drawer_left'][0],location_list['drawer_left'][1],location_list['drawer_left'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in tool_list:
                    omni_base.go_abs(location_list['drawer_right'][0],location_list['drawer_right'][1],location_list['drawer_right'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in food_list:
                    omni_base.go_abs(location_list['Tray_A'][0],location_list['Tray_A'][1],location_list['Tray_A'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in kitchen_list:
                    omni_base.go_abs(location_list['cotainers_A'][0],location_list['cotainers_A'][1],location_list['cotainers_A'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in oriantation_list:
                    omni_base.go_abs(location_list['cotainers_B'][0],location_list['cotainers_B'][1],location_list['cotainers_B'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in task_list:
                    omni_base.go_abs(location_list['bin_A'][0],location_list['bin_A'][1],location_list['bin_A'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)
                else:
                    omni_base.go_abs(location_list['bin_B'][0],location_list['bin_B'][1],location_list['bin_B'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                gripper.command(1.0)
                rospy.sleep(2.0)
                omni_base.go_abs(location_list['pick_A'][0],location_list['pick_A'][1],location_list['pick_A'][2])
                rospy.sleep(3.0)
                whole_body.move_to_go()
            omni_base.go_abs(location_list['pick_B'][0],location_list['pick_B'][1],location_list['pick_B'][2])
            rospy.sleep(3.0)

        while rospy.is_shutdown() == False:
            whole_body.move_to_go()
            #首を下に向ける
            whole_body.move_to_joint_positions({'head_tilt_joint': -0.8})
            rospy.sleep(3.0)
            #手を閉じる
            rospy.sleep(3.0)
            object_list = arm.take_list()
            #手を開ける
            rospy.sleep(3.0)
            gripper.command(1.0)
            rospy.sleep(3.0)
            if object_list == "no_object":
                print("no object")
                break
            else:
                #object_listの中身を発話
                print(object_list[0][0])
            
                try:
                    arm.instance_floor(object_list[0][0])
                    rospy.sleep(3.0)
                except:
                    print("error")
                    pass
                whole_body.move_to_go()
                rospy.sleep(3.0)
                
                if object_list[0][0] in shape_list:
                    rospy.sleep(5.0)
                    omni_base.go_abs(location_list['drawer_left'][0],location_list['drawer_left'][1],location_list['drawer_left'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in tool_list:
                    omni_base.go_abs(location_list['drawer_right'][0],location_list['drawer_right'][1],location_list['drawer_right'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)
                    
                elif object_list[0][0] in food_list:
                    omni_base.go_abs(location_list['Tray_A'][0],location_list['Tray_A'][1],location_list['Tray_A'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in kitchen_list:
                    omni_base.go_abs(location_list['cotainers_A'][0],location_list['cotainers_A'][1],location_list['cotainers_A'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in oriantation_list:
                    omni_base.go_abs(location_list['cotainers_B'][0],location_list['cotainers_B'][1],location_list['cotainers_B'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in task_list:
                    omni_base.go_abs(location_list['bin_A'][0],location_list['bin_A'][1],location_list['bin_A'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)
                else:
                    omni_base.go_abs(location_list['bin_B'][0],location_list['bin_B'][1],location_list['bin_B'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                gripper.command(1.0)
                rospy.sleep(2.0)
                omni_base.go_abs(location_list['pick_A'][0],location_list['pick_A'][1],location_list['pick_A'][2])
                rospy.sleep(3.0)
                whole_body.move_to_go()
            omni_base.go_abs(location_list['pick_B'][0],location_list['pick_B'][1],location_list['pick_B'][2])
            rospy.sleep(3.0)
            
        omni_base.go_abs(location_list['pick_C'][0],location_list['pick_C'][1],location_list['pick_C'][2])
        rospy.sleep(5.0)

        while rospy.is_shutdown() == False:
            whole_body.move_to_go()
            #首を下に向ける
            whole_body.move_to_joint_positions({'head_tilt_joint': -0.8})
            rospy.sleep(3.0)
            #手を閉じる
            rospy.sleep(3.0)
            object_list = arm.take_list()
            #手を開ける
            rospy.sleep(3.0)
            gripper.command(1.0)
            rospy.sleep(3.0)
            if object_list == "no_object":
                print("no object")
                break
            else:
                #object_listの中身を発話
                print(object_list[0][0])
            
                try:
                    arm.instance_floor(object_list[0][0])
                    rospy.sleep(3.0)
                except:
                    print("error")
                    pass
                whole_body.move_to_go()
                rospy.sleep(3.0)
                
                if object_list[0][0] in shape_list:
                    rospy.sleep(5.0)
                    omni_base.go_abs(location_list['drawer_left'][0],location_list['drawer_left'][1],location_list['drawer_left'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in tool_list:
                    omni_base.go_abs(location_list['drawer_right'][0],location_list['drawer_right'][1],location_list['drawer_right'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in food_list:
                    omni_base.go_abs(location_list['Tray_A'][0],location_list['Tray_A'][1],location_list['Tray_A'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in kitchen_list:
                    omni_base.go_abs(location_list['cotainers_A'][0],location_list['cotainers_A'][1],location_list['cotainers_A'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in oriantation_list:
                    omni_base.go_abs(location_list['cotainers_B'][0],location_list['cotainers_B'][1],location_list['cotainers_B'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in task_list:
                    omni_base.go_abs(location_list['bin_A'][0],location_list['bin_A'][1],location_list['bin_A'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)
                else:
                    omni_base.go_abs(location_list['bin_B'][0],location_list['bin_B'][1],location_list['bin_B'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                gripper.command(1.0)
                rospy.sleep(2.0)
                omni_base.go_abs(location_list['pick_C'][0],location_list['pick_C'][1],location_list['pick_C'][2])
                rospy.sleep(3.0)
                whole_body.move_to_go()
            rospy.sleep(3.0)
        


        return "to_taskB"
        
        


class TaskB(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['finish'])

    def execute(self,userdata):
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


def main():
    sm_tidy_up = smach.StateMachine(outcomes=['FINISH'])

    with sm_tidy_up:
        #EnterRoom
        smach.StateMachine.add('ENTER',EnterRoom(),
                transitions={'to_taskA':'TASK_A'})

        #Task A
        smach.StateMachine.add('TASK_A',TaskA(),
                               transitions={'to_taskB':'TASK_B'})

        #Task 2A
        #smach.StateMachine.add('TASK_2A',Task2A(),
        #        transitions={'to_taskB':'TASK_B'})

        #Task B 
        smach.StateMachine.add('TASK_B',TaskB(),
                transitions={'finish':'FINISH'})

        outcome = sm_tidy_up.execute()


if __name__ =='__main__':
    #rospy.init_node('sm_tidy_up')
    main()
