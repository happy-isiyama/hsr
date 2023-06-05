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

sys.path.append("/home/demulab/dspl_ws/src/hsr/robot/src")
#from module import *
from arm import *

#locatin_list 辞書型で定義
location_list = {'table_a':(2.284162159336223, 2.929703496804872, 0.061572093333072314),
                 'table_b':(2.380020627581534, 1.056500281377312, 0.02266108632762348),
                 'table_center':(1.5827454106605983, 1.8262875841168187, 3.129996744694669),
                 'table_left':(1.595966813767852, 1.2583708350487552, 3.120435860989875),
                 'table_right':(1.563144172541887, 2.383551753188647, 3.140204424235087),
                 'waypoint1':(2.439885828485936, 0.12257944264739871, 0.0452156824171235),
                 'waypoint2':(3.9091511519156223, 0.15497955104277086, 0.038638968574600786),
                 'shelf':(4.4331769989766485, 1.0486846497559235, 0.03786785857847326),
                 'human_center':(4.63224620012862, 1.4850550571247705, 1.609544631220165),
                 'human_left':(4.022050393438807, 2.2987666211105346, 1.6000028501606836),
                 'human_right':(5.053066552752714, 2.347473880985127, 1.5918783317735565)}


# define Instruction state
def Instruction():
    rospy.loginfo('Executing state Instruction')
    rospy.sleep(1)
    #指示を仰ぐ
    tts.say('Please tell me the task')
    #音声認識で指示を受け取る　指示はlist型で受け取る
    #認識できなかった場合は再度指示を仰ぐ
    while True:
        try:
            command = recog.recognize()
            break
        except:
            tts.say('I could not understand. Please tell me again')

    return command


# define Task state
def Task(command):
    rospy.loginfo('Executing state Task')
    rospy.sleep(1)
    #指示を実行する
    if command[0] == 'Go' and command[2] == 'grasp' and command[5] == "place":
        #Go to the ＄{部屋名}, grasp the ＄{物体名} on the ＄{場所名} and place it on the ＄{場所名}.
        #{部屋名}の場所に移動
        omnibase.go(location_list[command[1]][0], location_list[command[1]][1], location_list[command[1]][2])
        #{場所名}の場所に移動
        omnibase.go(location_list[command[4]][0], location_list[command[4]][1], location_list[command[4]][2])
        #物体を掴む
        arm.instance_desk(command[3])
        #{場所名}の場所に移動
        omnibase.go(location_list[command[6]][0], location_list[command[6]][1], location_list[command[6]][2])
        #物体を置く
        arm.place_object()


    elif command[0] == 'Go' and command[2] == 'grasp' and command[5] == "give":
        #Go to the ＄{部屋名}, grasp the ＄{物体名} on the ＄{場所名} and give it to ＄{人物名}.
        #{部屋名}の場所に移動
        omnibase.go(location_list[command[1]][0], location_list[command[1]][1], location_list[command[1]][2])
        #{場所名}の場所に移動
        omnibase.go(location_list[command[4]][0], location_list[command[4]][1], location_list[command[4]][2])
        #物体を掴む
        arm.instance_desk(command[3])
        #{人物名}の場所に移動
        omnibase.go(location_list[command[6]][0], location_list[command[6]][1], location_list[command[6]][2])
        #物体を渡す
        arm.give_object()

    
    elif command[0] == 'Go' and command[2] == 'find' and command[5] == "follow":
        #Go to the ＄{部屋名}, find ＄{人物名} at the ＄{場所名} and follow (him | her).
        omnibase.go(location_list[command[1]][0], location_list[command[1]][1], location_list[command[1]][2])
        omnibase.go(location_list[command[4]][0], location_list[command[4]][1], location_list[command[4]][2])
        #人物を探す
        #人物を追う

    elif command[0] == 'Go' and command[2] == 'find' and command[5] == "guide":
        #Go to the ＄{部屋名}, find ＄{人物名} at the ＄{場所名} and guide (him | her) to the ＄{場所名}.
        omnibase.go(location_list[command[1]][0], location_list[command[1]][1], location_list[command[1]][2])
        omnibase.go(location_list[command[4]][0], location_list[command[4]][1], location_list[command[4]][2])
        #人物を探す
        #人物を案内する

    elif command[0] == 'Go' and command[2] == 'find' and command[5] == "answer":
        #Go to the ＄{部屋名}, find ＄{人物名} at the ＄{場所名} and answer (his | her) question.
        omnibase.go(location_list[command[1]][0], location_list[command[1]][1], location_list[command[1]][2])
        omnibase.go(location_list[command[4]][0], location_list[command[4]][1], location_list[command[4]][2])
        #人物を探す
        #人物の質問に答える

    elif command[0] == 'Go' and command[2] == 'find' and command[5] == "ask":
        #Go to the ＄{部屋名}, find ＄{人物名} at the ＄{場所名} and ask (him | her) ＄{質問}.
        omnibase.go(location_list[command[1]][0], location_list[command[1]][1], location_list[command[1]][2])
        omnibase.go(location_list[command[4]][0], location_list[command[4]][1], location_list[command[4]][2])
        #人物を探す
        #人物に質問する

    elif command[0] == 'Tell' and command[1] != "people":
        #Tell me how many ＄{物体カテゴリー名} there are on the ＄{場所名}.
        #{場所名}の場所に移動
        omnibase.go(location_list[command[3]][0], location_list[command[3]][1], location_list[command[3]][2])
        #物体カテゴリー名の物体の数を数える

    elif command[0] == 'Tell' and command[1] == "people":
        #Tell me how many people in the ＄{部屋名} are ＄{ポーズ}.
        #{部屋名}の場所に移動
        omnibase.go(location_list[command[2]][0], location_list[command[2]][1], location_list[command[2]][2])
        #人物の数を数える

    else:
        #指示が間違っている
        tts.say('error')
        return 0


# define main 
def main():
    rospy.init_node('sm_gpsr')
    tts.say('start gpsr')
    #３回成功するまで繰り返す
    while True:
        command = Instruction()
        result = Task(command)
        if result == 1:
            count += 1
        if count == 3:
            break
    tts.say('finish gpsr')


if __name__ == '__main__':
    main()
