#!/usr/bin/python
# -*- coding: utf-8 -*-

#import ros
import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String
#import hsrb
from hsrb_interface import geometry
from hsrb_interface import Robot

#humanpose
sys.path.append("/home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose/src")
import cv2
import matplotlib.pyplot as plt
import copy
import numpy as np
import torch
sys.path.append("/home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose")
from src import model 
from src import util 
from src.body import Body 
from src.hand import Hand 
from cv_bridge import CvBridge

import hsrb_interface
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf

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
        self.listener = tf.TransformListener()
        print('pass')
        pass

    def instance_floor(self,object_name):
        topic_tf = object_name
        object_to_hand = geometry.pose(ei=-1.57)
        # handを0.1[m]上に移動させる姿勢
        hand_up = geometry.pose(z=-0.1)

        gripper.command(1.0)
        whole_body.move_to_neutral()
        whole_body.looking_hand_constraint = True
        rospy.sleep(5.0)
        whole_body.gaze_point(point=geometry.Vector3(y=0, x=0.8, z=0.5), ref_frame_id='base_link')
        self.listener.waitForTransform("head_rgbd_sensor_rgb_frame", topic_tf, rospy.Time(), rospy.Duration(4.0))
        whole_body.move_end_effector_pose(object_to_hand, topic_tf)
        rospy.sleep(2.0)
        # 力を指定して把持する
        gripper.apply_force(_GRASP_FORCE)
        # シミュレータのgrasp hackのための待ち時間。実機では不要
        rospy.sleep(2.0)
        # 手先相対で上にハンドを移動
        whole_body.move_end_effector_pose(hand_up, _HAND_TF)
        # 手先相対で後ろにハンドを移動
        # 初期姿勢に遷移
        whole_body.move_to_neutral()
        tts.say('把持に成功しました')
        gripper.command(1.0)
        sys.exit()

    def place_object(self):
        whole_body.looking_hand_constraint = False
        whole_body.move_to_neutral()
        rospy.sleep(2.0)
        gripper.apply_force(_GRASP_FORCE)
        whole_body.move_to_joint_positions({'arm_lift_joint': 0.4})
        rospy.sleep(1.0)
        whole_body.linear_weight = 100
        rospy.sleep(1.0)
        whole_body.move_end_effector_by_line((0, 0, 1), 0.2)
        #whole_body.linear_weight = 1.0
        rospy.sleep(1.0)
        #omni_base.go_abs(0,0,0)
        print(omni_base.pose)
        omni_base.go_rel(0.5, 0, 0.0, 300.0)
        rospy.sleep(5.0)
        whole_body.move_to_joint_positions({'arm_lift_joint': 0.25})

    def instance_desk(self,object_name):
        topic_tf = object_name
        object_to_hand = geometry.pose(z=-0.1, ek=-1.57)

        # handを0.1[m]上に移動させる姿勢
        hand_push = geometry.pose(z=0.1)
        hand_up = geometry.pose(x=0.1)
        # handを0.5[m]手前に移動させる姿勢
        hand_back = geometry.pose(z=-0.5)

        gripper.command(1.0)
        whole_body.move_to_neutral()
        whole_body.looking_hand_constraint = True
        whole_body.gaze_point(point=geometry.Vector3(x=1.0,y=0,z=0.5), ref_frame_id='base_link')
        self.listener.waitForTransform("head_rgbd_sensor_rgb_frame", topic_tf, rospy.Time(), rospy.Duration(4.0))
        rospy.sleep(3.0)
        whole_body.move_end_effector_pose(object_to_hand, topic_tf)
        # 力を指定して把持する
        whole_body.move_end_effector_pose(hand_push, _HAND_TF)
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
 
    def instance_shelf(self,object_name):
        topic_tf = object_name
        object_to_hand = geometry.pose(z=-0.1, ek=-1.57)

        # handを0.1[m]上に移動させる姿勢
        hand_push = geometry.pose(z=0.1)
        hand_up = geometry.pose(x=0.1)
        # handを0.5[m]手前に移動させる姿勢
        hand_back = geometry.pose(z=-0.5)

        gripper.command(1.0)
        whole_body.move_to_neutral()
        #whole_body.move_to_go()
        whole_body.move_to_joint_positions({'arm_lift_joint': 0.3})
        whole_body.looking_hand_constraint = True
        #whole_body.gaze_point(point=geometry.Vector3(x=1.0,y=0,z=0.5), ref_frame_id='base_link')
        self.listener.waitForTransform("head_rgbd_sensor_rgb_frame", topic_tf, rospy.Time(), rospy.Duration(4.0))
        rospy.sleep(3.0)
        whole_body.move_end_effector_pose(object_to_hand, topic_tf)
        # 力を指定して把持する
        whole_body.move_end_effector_pose(hand_push, _HAND_TF)
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



#humanposr src start
class OpenPose():
    def __init__(self):
        print("openpose")
        rospy.init_node("subscriber")
        rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image,self.callback)
        self.pub = rospy.Publisher("human2LR",String,queue_size = 10)
        self.msgLR = "false"
        #rospy.sleep()
        #self.oriImg = np.empty()
        

    def callback(self,msg):
        bridge = CvBridge()
        self.oriImg = bridge.imgmsg_to_cv2(msg)

    def humankamo(self):
        print("humanstart")
        body_estimation = Body('/home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose/model/body_pose_model.pth')
        hand_estimation = Hand('/home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose/model/hand_pose_model.pth')
        candidate, subset = body_estimation(self.oriImg)
        canvas = copy.deepcopy(self.oriImg)
        canvas,self.msgLR = util.draw_bodypose(canvas, candidate, subset, self.oriImg)

    # detect hand
        hands_list = util.handDetect(candidate, subset, self.oriImg)

        all_hand_peaks = []
        for x, y, w, is_left in hands_list:
            peaks = hand_estimation(self.oriImg[y:y+w, x:x+w, :])
            peaks[:, 0] = np.where(peaks[:, 0]==0, peaks[:, 0], peaks[:, 0]+x)
            peaks[:, 1] = np.where(peaks[:, 1]==0, peaks[:, 1], peaks[:, 1]+y)
            all_hand_peaks.append(peaks)

        canvas = util.draw_handpose(canvas, all_hand_peaks)

        cv2.imshow('human', canvas)

        rospy.sleep(1.0)
        #print(self.msgLR)
        return self.msgLR

    def humanposeLR(self):
        while self.msgLR == "false":
            rospy.sleep(1.0)
        return self.msgLR



