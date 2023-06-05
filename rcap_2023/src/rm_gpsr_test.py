#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import sys
import rospy
import smach
import smach_ros

#hsrb_interface
import hsrb_interface
from hsrb_interface import Robot, geometry
from sensor_msgs.msg import LaserScan

sys.path.append("/home/demulab/dspl_ws/src/hsr/robot/src")
#from module import *
#from arm import *
from humanmodule import *
from audiomodule import *
from manipulation import *

robot = hsrb_interface.Robot()
tts = robot.get("default_tts")
whole_body = robot.get("whole_body")
omni_base = robot.get("omni_base")

#locatin_list 辞書型で定義

location_list ={'entrance':(0.7593438613930832, -0.06262084319702434, 0.04509086597375952),
                'enter':(1.2665689822656732, -0.03874379822393036, -1.7258606588536878),
                'drawer right':(1.5576360589486657, -0.08550827985493153, -1.710737975204823),
                'drawer left':(2.0091599143348717, -0.38741638952841073, -1.7048507692120505),
                'containers A':(2.252136128072331, -0.4537567645813348, -1.72185399079363),
                'containers B':(2.352136128072331, -0.4537567645813348, -1.72185399079363),
                'Trays A':(2.5121977147517898, -0.47434614276926423, -1.7225263036446665),
                'Trays B':(2.879051476095184, -0.48258444644527826, -1.54868145211231),
                'bin A':(3.3934779358575886, -0.5544480852367322, -1.5619447096514851),
                'bin B':(3.960155677581472, -0.5291122666830548, -1.6222167495189816),
                'pick A':(0.9812442860658123, 0.4591283889671645, 1.5331071660493234),
                'pick B':(1.671502283990401, 0.5034319187174363, 1.4219231901676603),
                'pick C':(2.5305265301234625, 0.5296583412704511, 1.463515796810405),
                'table B':(2.1037579497598538, 1.2794696599556121, 1.5249952219681961),
                'living room':(3.698556817104813, 0.1579836268110597, 1.6062332436121731),
                'task2a_start':(3.700136780324349, 1.9313400273376218, 1.5213156770688085),
                'task2a_end':(3.6629467653550996, 3.692564971029578, 1.4208711519579684),
                'shelf right':(3.2595201101875912, 3.982557296669328, 1.552463855003606),
                'shelf center':(3.0643157685051796, 3.9576666351181777, 1.576485633243085),
                'shelf left':(2.801047491189583, 3.9277634025740573, 1.5072258200410695),
                'dining room':(2.3467239615914743, 3.5442556901153144, 3.113921765258759),
                'human left':(1.583878891773181, 3.0747168094238257, 3.0842118719946323),
                'human right':(1.6380188914519145, 3.9927658472039997, 3.133774307508016)}
arm = Arm()

class State1():
    def __init__(self):
        print("dspl")
        self.Do_list = []
        self.text = String()
        self.count = 1
        self.how = 0
        #self.scan_subscriber = rospy.Subscriber('/hsrb/base_scan', LaserScan, self.scan_callback)
    def voice(self):
        Gp = Gpsr()
        self.text,self.Do_list,self.how = Gp.Voice()
        if self.Do_list[0] == "Go":
            print("Go")
            for a in range(self.how):
                print(self.Do_list[a])
                if self.Do_list[a] == "grasp":
                    for b in range(self.how):
                        if self.Do_list[5] == "place":
                            tts.say(str(self.text))
                            rospy.sleep(2)
                            self.place()
                        elif self.Do_list[5] == "give":
                            rospy.sleep(2)
                            self.give()
                        else:
                            tts.say("すみません、もう一度お願いします")
                            rospy.sleep(1)
                            self.voice()
                elif self.Do_list[a] == "find":
                    for c in range(self.how):
                        if self.Do_list[c] == "follow":
                            tts.say(str(self.text))
                            rospy.sleep(2)
                            self.follow()
                        elif self.Do_list[c] == "guide":
                            guide(self.Do_list,self.count)
                        elif self.Do_list[c] == "answer":
                            tts.say(str(self.text))
                            rospy.sleep(2)
                            self.answer()
                        elif self.Do_list[c] == "ask":
                            tts.say(str(self.text))
                            rospy.sleep(2)
                            self.ask()
                        else:
                            tts.say("すみません、もう一度お願いします")
                            self.voice()
        elif self.Do_list[0] == "Tell":
            tts.say(str(self.text))
            rospy.sleep(2)
            tts.say("違う質問をお願いします")
        else:
            self.voice()

    def place(self):
        #placeのときのコードを書く
        print("place")
        omni_base.go_abs(location_list[command[self.Do_list[1]]][0],location_list[command[self.Do_list[1]]][1],location_list[command[self.Do_list[1]]][2])
        rospy.sleep(0.5)
        #arm = Arm()
        tts.say(str(self.Do_list[3])+"をつかみます")
        result = arm.instance_desk(self.Do_list[3])
        if result == "True":
            rospy.sleep(0.5)
            tts.say(str(self.Do_list[7]+"に移動します"))
            rospy.sleep(0.5)
            omni_base.go_abs(location[command[self.Do_list[7]]][0],location_list[command[self.Do_list[7]]][1],location_list[command[self.Do_list[7]]][2])
            #rospy.sleep(0.5)
            rospy.sleep(1)
            tts.say(str(self.Do_list[3])+"を置きます")
            arm.instance_place_object2(self.Do_list[3])
            rospy.sleep(0.5)
            tts.say("ホームに戻ります")
        #arm.place_object()
        self.count =+ 1
        self.main()
    def give(self):
        #giveのときのコードを書く
        for f in range(22):
            if location_list[command[f]] == self.Do_list[1]:
                omnibase.go_abs(location_list[command[f]][0],location_list[command[f]][1],location_list[command[f]][2])
                rospy.sleep(0.5)
                tts.say(str(self.Do_list[3])+"をつかみます")
                #arm = Arm()
                rospy.sleep(0.5)
                #arm.instance_floor(Do_list[3])
                #目的の人に手を上げてもらい移動する
                tts.say(str(self.Do_list[6])+"HSRの前に来てください")
                rospy.sleep(0.5)
                #ワンちゃん人を認識する必要あるかも
                tts.say(str(self.Do_list[3])+"を渡します")
                #物を離したい
            else:
                pass
        self.count =+ 1
        self.main()
    def follow(self):
        #followのときのコードを書く
        for g in range(22):
            if location_list[command[g]]== self.Do_list[1]:
                omnibase.go_abs(location_list[command[g]][0],location_list[command[g]][1],location_list[command[g]][2])
                #目的の人を探す
                print("目的の人を探す")
            else:
                pass
        self.count =+ 1
        self.main()
    def guide(self):
        #guideのときのコードを書く
        for h in range(22):
            if location_list[command[h]] == self.Do_list[1]:
                omnibase.go_abs(location_list[command[h]][0],location_list[command[h]][1],location_list[command[h]][2])
                #目的の人を探し移動する
                for i in range(22):
                    if location_list[command[i]] == self.Do_list[7]:
                        omnibase.go_abs(location_list[command[i]][0],location_list[command[i]][1],location_list[command[i]][2])
            else:
                pass
        self.count =+ 1
        self.main()
    def answer(self):
        questions = []
        #answerのときのコードを書く
        #for j in range(11):
            #if location_list[command[j] == self.Do_list[1]:
                #omnibase.go(location_list[command[j]][0],location_list[command[j]][1],location_list[command[j]][2])
        questions = self.text.splitlines("and")
        if questions[1] == "What is the highest mountain in Japan":
            tts.say("It is Mt.Fuji") 
            rospy.sleep(0.5)
        elif questions[1] == "What is the largest lake in Japan":
            rospy.sleep(0.5)
            tts.say("It is Lake Biwa")
            rospy.sleep(0.5)
        elif questions[1] == "Where are you coming from":
            tts.say("We are coming from Ishikawa prefecture")
            rospy.sleep(0.5)
        elif questions[1] == "Are you enjoying this competition":
            tts.say("Yes, we are enjoying it a lot.")
            rospy.sleep(0.5)
        elif questions[1] == "What is your favorite drink":
            tts.say("My favorite drink is coffe")
        elif questions[1] == "How are you today":
            tts.say("I am feeling exciting")
        elif questions[1] == "Do Thai people ride an elephant to go to the university":
            tts.say("Yes, Sometimes the elephant oversleeps and I'm late, though ")
        elif questions[1] == "Which country won the WBC this year":
            tts.say("Japan!!!")
        elif questions[1] == "How many joints does your robot arm have":
            tts.say("It has 5")
        elif questions[1] == "What is 3 times 5":
            tts.say("It is 15")
        else:
            tts.say("one more please")
            self.voice()
        self.count =+ 1
        self.main()
    def ask(self):
        #askのときのコードを書く
        for k in range(22):
            if location_list[command[k]] == self.Do_list[1]:
                omnibase.go_abs(location_list[command[k]][0],location_list[command[k]][1],location_list[command[k]][2])
                #目的の人を探し移動する
                #質問をする
            else:
                pass
        self.count =+ 1
        self.main()

    def main(self):
        rospy.sleep(2)
        if self.count == 4:
            tts.say("end")
            pass
        elif self.count == 1:
            tts.say("start GPSR")
            rospy.sleep(1)
            tts.say("ドアを開けてください")
            rospy.sleep(5)
            #omni_base.go_rel(1.5,0.0,0.0,300.0)
            #self.door()
            print("voice")
            self.voice()
        else:
            omni_base.go(0,0,0)
            self.voice()
            
    def scan_callback(msg):
        #global scan_subscriber  # scan_subscriber をグローバル変数として宣言する
        if min(msg.ranges) < 0.5:
            print("Obstacle detected! Start again.")
        # ここで再びサブスクライバーを解除する
            #self.scan_subscriber.unregister()
            rospy.sleep(3)
            tts.say("ドアを開けてください")
            rospy.sleep(3)
            tts.say("Please open the door")
            #self.door()
        # 再びサブスクライバーを作成する
        #scan_subscriber = rospy.Subscriber('/hsrb/base_scan', LaserScan, scan_callback)
    
        else:
            tts.say("進みます")
            rospy.sleep(3)
            tts.say("Move on")
            print("No obstacle detected. Move forward.")
            omni_base.go_abs(3.0, 0.0, 0.0)
            tts.say("終わり")
            rospy.sleep(3)
            tts.say("finished")
            rospy.signal_shutdown("Finished moving.")

    def door():
        #global scan_subscriber  # scan_subscriber をグローバル変数として宣言する
    #rospy.init_node('obstacle_detection', anonymous=True)
    #omni_base.go_abs(0.0, 0.0, 0.0) あってもなくても
        rospy.sleep(1)
    #tts.say("初期位置移動") あってもなくても
        #rospy.spin()

if __name__ == "__main__":
    st = State1()
    st.main()
