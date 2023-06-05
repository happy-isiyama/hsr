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
from sensor_msgs.msg import LaserScan


sys.path.append("/home/demulab/dspl_ws/src/hsr/robot/src")
#from module import *
from manipulation import *
from humanmodule import *
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
arm = Arm()

shape_list = ["soccer_ball","Soft_ball","base_ball",
              "tennis_ball","racquet_ball","golf_ball","stacking_blocks","brick","dice","rope","chain"]
tools_list =["pen","Keys","clamp"]
food_list = ["cracker_box","box_of_sugar","chocolate_pudding",
             "gelatin_box","meat_can","coffee",
             "tuna_can","Pringles chips can","French’s mustard bottle",
             "Tomato soup can","Plastic banana","Plastic strawberry","Plastic apple",
             "Plastic lemon","Plastic peach","Plastic pear","Plastic orange","Plastic plum"]

kitchen_list = ["Windex Spray bottle","Srub cleanser bottle","Scotch brite dobie sponge",
                "Pitcher base","Pitcher lid","Plate","Bowl","Fork","Spoon","Spatula",
                "Wine glass","Mug"]
orientatio_based_list = ["Credit card blank","stacking_blocks",]
task_items = ["lego","wood_blocks","9-peg-hole test","Toy airplane","Lego duplo","Magazine","Black t-shirt","Timer"]


#locatin_list 辞書型で定義
location_list= {
        'entrance':(-0.00152473042404359, 0.001715338930608372, -0.001425828274587242),
        'drawer_right':(1.6445146869600311, -0.06772295188720022, -1.6575697761472434),
        'drawer_left':(2.0028786187053687, -0.138483521031794, -1.5837886493421804),
        'long_table_a1':(2.4993291117634855, -0.1830820880171906, -1.5962232820358395),
        'long_table_a2':(2.767177050517386, -0.18437988454847526, -1.5920303358505248),
        'long_table_a3':(3.1108736737357248, -0.20116547929314849, -1.6048323315351563),
        'long_table_a4':(3.395171294310287, -0.23893123063440747, -1.6227333590907962),
        'bin_1':(3.874159662304694, -0.2526188384942371, -1.6125261596850666),
        'bin_2':(4.36971088969307, -0.28847385507221074, -1.648560185920845),
        'tall_table':(1.6343730368220897, 0.12545452642871052, 1.495261682299534),
        'long_table_b1':(2.3033910618291835, 0.09211335244021301, 1.5328669295127966),
        'long_table_b2':(2.861026195350513, 0.24618715008357794, 1.5349937069028603),
        'task1_area':(2.312394986951096, 0.9919030600300746, 1.48172722300258),
        'task2_area_1':(4.145697040449563, 1.0441225821491489, 1.3817288044445386),
        'task2_area_2':(4.2645874642504875, 3.2433043748665225, 1.3339297615108099),
        'shelf':(3.9367202136557657, 3.4481489992767336, 1.5655760155898204),
        'chair_center':(3.1538932404744138, 3.36003486612061, 3.1249827953751455),
        'chair_right':(2.1296058174460604, 2.8644017849871535, 3.0646812872056013),
        'chair_left':(2.0866085523531934, 4.0378451076290585, 3.0313597292110166)}


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
        print(self.range[160])
        while self.range[160] < 0.5:
            
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
        smach.State.__init__(self,outcomes=['to_task2A'])
        print()
    #call back function for the subscriber of the topic /appear_list String
    #bottle,1.2269999980926514,chair,4.8420000076293945,backpack,5.210000038146973"のような文字列が入ってくる
        #print(self.appear_list)i

    def execute(self, userdata):
        rospy.loginfo("----------------Task A--------------------")
        #talle tableに向かう


        omni_base.go_abs(location_list['tall_table'][0],location_list['tall_table'][1],location_list['tall_table'][2], 300.0)
        rospy.sleep(5.0)

        while rospy.is_shutdown() == False:
            whole_body.move_to_go()
            #首を下に向ける
            whole_body.move_to_joint_positions({'head_tilt_joint': -0.8})
            rospy.sleep(3.0)
            #手を閉じる
            gripper.command(0.0)
            rospy.sleep(3.0)
            object_list = arm.take_list()
            print(object_list)
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
                
                if object_list[0][0] in ["cola"]:
                    rospy.slee:p(5.0)
                    ###omni_base.go_abs(location_list['drawer_left'][0],location_list['drawer_left'][1],location_list['drawer_left'][2])
                    rospy.sleep(2.0)
                    arm.place_object()
                    rospy.sleep(2.0)

                elif object_list[0][0] in ["tea"]:
                    ###omni_base.go_abs(location_list['drawer_right'][0],location_list['drawer_right'][1],location_list['drawer_right'][2])
                    rospy.sleep(2.0)
                    arm.place_object()
                    rospy.sleep(2.0)

                elif object_list[0][0] in ["pocky"]:
                    ###omni_base.go_abs(location_list['long_table_a3'][0],location_list['long_table_a3'][1],location_list['long_table_a3'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in ["takenoko"]:
                    ###omni_base.go_abs(location_list['long_table_a1'][0],location_list['long_table_a3'][1],location_list['long_table_a1'][2])
                    rospy.sleep(2.0)
                    arm.place_object2()
                    rospy.sleep(2.0)

                elif object_list[0][0] in ["noodle"]:
                    ###omni_base.go_abs(location_list['long_table_a2'][0],location_list['long_table_a2'][1],location_list['long_table_a2'][2])
                    rospy.sleep(2.0)
                    arm.place_object()
                    rospy.sleep(2.0)
	return "to_task2A"

class Task2A(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['to_taskB'])

    def execute(self,userdata):
        rospy.loginfo("----------------Task 2A--------------------")
    
        #rospy.sleep(5.0)
        ###omni_base.go_abs(location_list['task2_area_1'][0],location_list['task2_area_1'][1],location_list['task2_area_1'][2])
        rospy.sleep(5.0)
        object_list = arm.take_list()
        print(omni_base.pose)
        #object_listがno_objectではないとき
        if object_list != "no_object":
        #相対座標で1.5m前に移動
            try:
                whole_body.angular_weight = 80.0
                whole_body.move_end_effector_pose(geometry.pose(x=3.0, z=0.7) ,ref_frame_id='map')
                rospy.sleep(1.0)
            
            except rospy.ROSInterrup:
                print("error")
                pass
        omni_base.go_rel(3.0,0.12,0.043,100.0)
        rospy.sleep(5.0)
        return "to_taskB"

class TaskB(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['finish'])

    def execute(self,userdata):
        rospy.loginfo("----------------Task B--------------------")
        omni_base.go_abs(location_list['shelf'][0],location_list['shelf'][1],location_list['shelf'][2], 300.0)
        rospy.sleep(5.0)

        whole_body.move_to_go()
        #rospy.sleep(1.0)
        #whole_body.move_to_joint_positions({'head_tilt_joint': -1.2}) 
        rospy.sleep(1.0)
        object_list = arm.take_list()
        pick_object = 'noodle'
        print(omni_base.pose)
        #object_listがno_objectではないとき
        if object_list != "no_object":
            arm.shelf_collision(object_list,pick_object,0.02,0.05,0.3)
            rospy.sleep(1.0)
            try:
                arm.instance_shelf(pick_object)
                rospy.sleep(1.0)
            except rospy.ROSInterrup:
                print("error")
                pass
        rospy.sleep(1.0)
        ###omni_base.go_abs(location_list['chair_center'][0],location_list['chair_center'][1],location_list['chair_center'][2])
        rospy.sleep(2.0)
        result = "false"
        op=OpenPose()
        while result != left or result != right:
            tts.say('please raise your hands')
            rospy.sleep(2.0)
            op=OpenPose()
            result = op.human()
            rospy.sleep(1.0)
        if result == "left":
            ###omni_base.go_abs(location_list['chair_left'][0],location_list['chair_left'][1],location_list['chair_left'][2])
            rospy.sleep(2.0)
            arm.place_object()
            gripper.command(1.0)
            rospy.sleep(2.0)
        elif result == "right":
            ###omni_base.go_abs(location_list['chair_right'][0],location_list['chair_right'][1],location_list['chair_right'][2])
            rospy.sleep(2.0)
            arm.place_object()
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
                transitions={'to_task2A':'TASK_2A'})

        #Task 2A
        smach.StateMachine.add('TASK_2A',Task2A(),
                transitions={'to_taskB':'TASK_B'})

        #Task B 
        smach.StateMachine.add('TASK_B',TaskB(),
                transitions={'finish':'FINISH'})

        outcome = sm_tidy_up.execute()


if __name__ =='__main__':
    #rospy.init_node('sm_tidy_up')
    main()
