#!/usr/bin/python
# -*- coding: utf-8 -*-

from hsrb_interface import Robot
import rospy
from hsrb_interface import geometry
import sys

# 移動のタイムアウト[s]
_MOVE_TIMEOUT=60.0
# 把持力[N]
_GRASP_FORCE=0.2
# ボトルのtf名
#_BOTTLE_TF='ar_marker/201'
# グリッパのtf名
_HAND_TF='hand_palm_link'
# ロボット機能を使うための準備
robot = Robot()
base = robot.try_get('omni_base')
tts = robot.try_get('default_tts')
whole_body = robot.try_get('whole_body')




class Arm():
    def __init__(self):
        print('pass')
        pass

    def ar_catch(self,ar):
        topic_tf = 'ar_marker/' + ar
        object_to_hand = geometry.pose(z=-0.02, ek=-1.57)
        
        # handを0.1[m]上に移動させる姿勢
        hand_up = geometry.pose(x=0.1)
        

        gripper.command(1.0)
        whole_body.move_to_neutral()
        whole_body.looking_hand_constraint = True
        try:
            whole_body.move_end_effector_pose(object_to_hand, topic_tf)
            # 力を指定して把持する
            gripper.apply_force(_GRASP_FORCE)
            # シミュレータのgrasp hackのための待ち時間。実機では不要
            rospy.sleep(2.0)
            # 手先相対で上にハンドを移動
            whole_body.move_end_effector_pose(hand_up, _HAND_TF)
            # 手先相対で後ろにハンドを移動
            whole_body.move_end_effector_pose(hand_back, _HAND_TF)
            # 初期姿勢に遷移
            whole_body.move_to_neutral()
            tts.say('把持に成功しました')

        except:
            tts.say('把持失敗')
            rospy.logerr('faile to grasp')
            sys.exit()


