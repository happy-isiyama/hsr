#!/usr/bin/python3
# -*- coding: utf-8 -*-

import hsrb_interface
from hsrb_interface import Robot
import rospy
from hsrb_interface import geometry
import sys
import tf2_ros

# 移動のタイムアウト[s]
_MOVE_TIMEOUT=60.0
# 把持力[N]
_GRASP_FORCE=0.2
# ボトルのtf名
#_BOTTLE_TF='ar_marker/201'
# グリッパのtf名
_HAND_TF='hand_palm_link'

# ロボット機能を使うための準備
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
gripper = robot.get('gripper')
tts = robot.get('default_tts')


class Arm():
    def __init__(self):
        print('pass')
        pass

    def ar_desk(self,ar):
        topic_tf = 'ar_marker/' + ar
        object_to_hand = geometry.pose(z=0, ek=-1.57)
        
        # handを0.1[m]上に移動させる姿勢
        hand_up = geometry.pose(x=0.1)
        # handを0.5[m]手前に移動させる姿勢
        hand_back = geometry.pose(z=-0.5)

        gripper.command(1.0)
        whole_body.move_to_neutral()
        whole_body.looking_hand_constraint = True
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
        gripper.command(1.0)
        sys.exit()


    def ar_floor(self,ar):
        topic_tf = 'ar_marker/' + ar
        pre_to_hand = geometry.pose(z=0,ek = -1.57)
       # object_to_hand = geometry.pose(x=-0.2, ek=-1.57)
        # handを0.1[m]上に移動させる姿勢
        hand_up = geometry.pose(x=-0.5)
        gripper.command(1.0)
        whole_body.move_to_neutral()
        whole_body.looking_hand_constraint = True

        looklist = [0.8,0.9,1.0,1.1,1.2,1.3]
        for i in range(len(looklist)):
            whole_body.gaze_point(point=geometry.Vector3(x=looklist[i], y=-0.5, z=0.3), ref_frame_id='base_link')
            try:
                rospy.sleep(2.0)
                whole_body.move_end_effector_pose(pre_to_hand, topic_tf)
                rospy.sleep(2.0)
                whole_body.move_end_effector_pose(object_to_hand, topic_tf)
                # 力を指定して把持する
                gripper.apply_force(_GRASP_FORCE)
                # シミュレータのgrasp hackのための待ち時間。実機では不要
                rospy.sleep(2.0)
                # 手先相対で上にハンドを移動
                whole_body.move_end_effector_pose(hand_up, _HAND_TF)
                # 手先相対で後ろにハンドを移動
                whole_body.move_to_neutral()
                tts.say('把持に成功しました')
                gripper.command(1.0)
                sys.exit()

            except:
                pass




if __name__=='__main__':
    arm = Arm()
    arm.ar_floor('201')
