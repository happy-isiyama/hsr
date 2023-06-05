#!/usr/bin/python
# -*- coding: utf-8 -*-

#import ros
import rospy
import sys
from sensor_msgs.msg import Image
from std_msgs.msg import String
'''
#import hsrb
import hsrb_interface
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
'''
'''
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf
'''
sys.path.append("/home/demulab/dspl/src/hsr/whisper/src")
import pyaudio
import wave
import whisper
'''
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
'''

'''
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
'''

#whisper yes or no

class Yesno():
     def __init__():
         print("voice yes or no")
         model = whisper.load_model("small.en")

     def voiceyn():
         model = whisper.load_model("small.en")
         CHUNK = 1024
         FORMAT = pyaudio.paInt16
         CHANNELS = 1
         RATE = 44100
         RECORD_SECONDS = 5
         WAVE_OUTPUT_FILENAME = "output.wav"

         p = pyaudio.PyAudio()

         stream = p.open(format = FORMAT,
                    channels = CHANNELS,
                    rate = RATE,
                    input = True,
                    #output = True,
                    #input_device_index = 2,
                    #output_device_index = 1,
                    frames_per_buffer = CHUNK)

         print("recording")

         frames = []

         for i in range(0,int(RATE / CHUNK * RECORD_SECONDS)):
             d = stream.read(CHUNK)
             frames.append(d)

         print("done recording")

         stream.stop_stream()
         stream.close()
         p.terminate()

         wf = wave.open(WAVE_OUTPUT_FILENAME,'wb')
         wf.setnchannels(CHANNELS)
         wf.setsampwidth(p.get_sample_size(FORMAT))
         wf.setframerate(RATE)
         wf.writeframes(b''.join(frames))
         wf.close()

     def recognizeyn():
         model = whisper.load_model("small.en")
         word = "String"
         audio = whisper.load_audio("output.wav")
         audio = whisper.pad_or_trim(audio)
         dictionary = ["yes","no","イエス","ノー","Yes","No"]
         yesdic = ["Yes","yes","イエス"]
         nodic = ["No","no","ノー"]

         mel = whisper.log_mel_spectrogram(audio).to(model.device)

         _,probs = model.detect_language(mel)
         print(f"Detected language:{max(probs,key=probs.get)}")

         options = whisper.DecodingOptions(fp16 = False)
         result = whisper.decode(model, mel, options)
         words = result.text.split()
         matched_words = []
         print(result.text)
         '''
         for word in words:
             if word.lower() in dictionary:
                 matched_words.append(word)

         print("Match:",matched_words)
         '''
         #print(type(result.text))
         if "Yes" in result.text:
             text = "Yes"
         elif "yes" in result.text:
             text = "Yes"   
         elif "No" in result.text:
             text = "No"
         elif "no" in result.text:
             text = "No"
         else:
             text = "False"

         return text
     
     def voicekamo():
         text = String()
         Yesno.voiceyn()
         text = Yesno.recognizeyn()

         return text

#食品認識

class food():
     def __init__(self):
         model = whisper.load_model("small")

     def voicefd():
         model = whisper.load_model("small")
         CHUNK = 1024
         FORMAT = pyaudio.paInt16
         CHANNELS = 1
         RATE = 44100
         RECORD_SECONDS = 5
         WAVE_OUTPUT_FILENAME = "output.wav"

         p = pyaudio.PyAudio()

         stream = p.open(format = FORMAT,
                    channels = CHANNELS,
                    rate = RATE,
                    input = True,
                    #output = True,
                    #input_device_index = 2,
                    #output_device_index = 1,
                    frames_per_buffer = CHUNK)

         print("recording")

         frames = []

         for i in range(0,int(RATE / CHUNK * RECORD_SECONDS)):
             d = stream.read(CHUNK)
             frames.append(d)

         print("done recording")

         stream.stop_stream()
         stream.close()
         p.terminate()

         wf = wave.open(WAVE_OUTPUT_FILENAME,'wb')
         wf.setnchannels(CHANNELS)
         wf.setsampwidth(p.get_sample_size(FORMAT))
         wf.setframerate(RATE)
         wf.writeframes(b''.join(frames))
         wf.close()

     def recognizefd():
         model = whisper.load_model("small")
         foodlist = []
         #foodtext = String()
         word = "String"
         audio = whisper.load_audio("output.wav")
         audio = whisper.pad_or_trim(audio)
         dictionary = ["cola","Cola","green tea","Green Tea","cup","Cup"]


         mel = whisper.log_mel_spectrogram(audio).to(model.device)

         _,probs = model.detect_language(mel)
         print(f"Detected language:{max(probs,key=probs.get)}")

         options = whisper.DecodingOptions(fp16 = False)
         result = whisper.decode(model, mel, options)
         words = result.text.split()
         matched_words = []
         print(result.text)
         '''
         for word in words:
             if word.lower() in dictionary:
                 matched_words.append(word)

         print("Match:",matched_words)
         '''
         if "Cola" in result.text:
             #foodtext = "Cola"
             foodlist.append("Cola")
         elif "cola" in result.text:
             #foodtext = "Cola"
             foodlist.append("Cola")
         elif "コーラ" in result.text:
             foodlist.append("Cola")
         if "Tea" in result.text:
             #foodtext = "Green Tea"
             foodlist.append("Green Tea")
         elif "tea" in result.text:
             foodlist.append("Green Tea")
         elif "ティー" in result.text:
             foodlist.append("Green Tea")
         else:
             foodlist.append("False")

         return foodlist

     def voicekana():
         fod = "String"
         food.voicefd()
         fod = food.recognizefd()

         return fod
