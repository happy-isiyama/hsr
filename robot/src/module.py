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

#humanposr src start
class OpenPose():
    def __init__(self):
        print("openpose")
        rospy.init_node("subscriber")
        rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image,self.callback)
        self.pub = rospy.Publisher("human2LR",String,queue_size = 10)
        #self.LRmsg = String
        pass

    def callback(self,msg):
        body_estimation = Body('home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose/model/body_pose_model.pth')
        hand_estimation = Hand('home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose/model/hand_pose_model.pth')

        print(f"Torch device: {torch.cuda.get_device_name()}")
#rospy.init_node("subscriber")
#rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image,callback)
    #cap = cv2.VideoCapture(0)
    #cap.set(3, 640)
    #cap.set(4, 480)

    #ret, oriImg = cap.read()
        oriImg = msg.data
        candidate, subset = body_estimation(oriImg)
        canvas = copy.deepcopy(oriImg)
        canvas,self.LRmsg = util.draw_bodypose(canvas, candidate, subset, oriImg)

    # detect hand
        hands_list = util.handDetect(candidate, subset, oriImg)

        all_hand_peaks = []
        for x, y, w, is_left in hands_list:
            peaks = hand_estimation(oriImg[y:y+w, x:x+w, :])
            peaks[:, 0] = np.where(peaks[:, 0]==0, peaks[:, 0], peaks[:, 0]+x)
            peaks[:, 1] = np.where(peaks[:, 1]==0, peaks[:, 1], peaks[:, 1]+y)
            all_hand_peaks.append(peaks)

        canvas = util.draw_handpose(canvas, all_hand_peaks)

        cv2.imshow('human', canvas)

        rospy.sleep(1.0)

        return self.LRMsg
        
#humanpose src end
