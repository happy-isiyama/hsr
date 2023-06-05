import sys
import rospy
sys.path.append("/home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose/src")
import cv2
import matplotlib.pyplot as plt
import copy
import numpy as np
import torch
import math
sys.path.append("/home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose")
from src import model 
from src import util 
from src.body import Body 
from src.hand import Hand 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import tf
class OpenPose():
    def __init__(self):
        print("openpose")
        #rospy.init_node("subscriber")
        #rospy.wait_for_message("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image)
        
        rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image,self.callback)
        #rospy.Subscriber("/camera/color/image_raw",Image,self.callback)

        #rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image,self.callback)
        rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image", Image, self.depthCB)
        #rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depthCB)
        #self.pub = rospy.Publisher("human2LR",String,queue_size = 10)
        self.br = tf.TransformBroadcaster()
        self.msgLR = "false"
        self.width = 640
        self.height = 480
        self.hfov = math.radians(58)#xtion 58)
        self.vfov = math.radians(45)#xtion 45)

    def depthCB(self,dep):
        #print("depth")
        bridge = CvBridge()
        self.dep = bridge.imgmsg_to_cv2(dep)

    def callback(self,msg):
        #print("callback")
        bridge = CvBridge()
        self.oriImg = bridge.imgmsg_to_cv2(msg)

    def human(self):
        body_estimation = Body('/home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose/model/body_pose_model.pth')
        candidate, subset = body_estimation(self.oriImg)
        canvas = copy.deepcopy(self.oriImg)
        canvas,self.msgLR,xpoint,ypoint = util.draw_bodypose(canvas, candidate, subset, self.oriImg)
        m_depth = self.dep[ypoint][xpoint]
        #print(xpoint)
        #print(ypoint)
        
        h_angle = (xpoint - self.width/2) * (self.hfov/self.width)
        v_angle = (ypoint - self.height/2) * (self.vfov/self.height)

        x = m_depth * math.sin(h_angle) * -1
        y = m_depth * math.sin(v_angle) * 1
        z = m_depth * 1
        self.br.sendTransform((x, y, z),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "otamesi",
                              "head_rgbd_sensor_link")

        return self.msgLR

    def humankamo(self):
        body_estimation = Body('/home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose/model/body_pose_model.pth')
        candidate, subset = body_estimation(self.oriImg)
        canvas = copy.deepcopy(self.oriImg)
        canvas,self.msgLR,xpoint,ypoint = util.draw_bodypose(canvas, candidate, subset, self.oriImg)
        m_depth = self.dep[ypoint][xpoint]
        #print(xpoint)
        #print(ypoint)

        h_angle = (xpoint - self.width/2) * (self.hfov/self.width)
        v_angle = (ypoint - self.height/2) * (self.vfov/self.height)

        x = m_depth * math.sin(h_angle) * -1
        y = m_depth * math.sin(v_angle) * 1
        z = m_depth * 1
        self.br.sendTransform((x, y, z),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "otamesi",
                              "head_rgbd_sensor_link")

        return x,z

    def humankana(self):
        body_estimation = Body('/home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose/model/body_pose_model.pth')
        candidate, subset = body_estimation(self.oriImg)
        canvas = copy.deepcopy(self.oriImg)
        canvas,self.msgLR,xpoint,ypoint = util.draw_bodypose(canvas, candidate, subset, self.oriImg)
        m_depth = self.dep[ypoint][xpoint]
        #print(xpoint)
        #print(ypoint)

        h_angle = (xpoint - self.width/2) * (self.hfov/self.width)
        v_angle = (ypoint - self.height/2) * (self.vfov/self.height)

        x = m_depth * math.sin(h_angle) * -1
        y = m_depth * math.sin(v_angle) * 1
        z = m_depth * 1
        self.br.sendTransform((x, y, z),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "otamesi",
                              "head_rgbd_sensor_link")

        return h_angle

    def humankanematu(self):
        body_estimation = Body('/home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose/model/body_pose_model.pth')
        candidate, subset = body_estimation(self.oriImg)
        canvas = copy.deepcopy(self.oriImg)
        canvas,self.msgLR,xpoint,ypoint = util.draw_bodypose(canvas, candidate, subset, self.oriImg)
        m_depth = self.dep[ypoint][xpoint]
        #print(xpoint)
        #print(ypoint)

        h_angle = (xpoint - self.width/2) * (self.hfov/self.width)
        v_angle = (ypoint - self.height/2) * (self.vfov/self.height)

        x = m_depth * math.sin(h_angle) * 1
        y = m_depth * math.sin(v_angle) * 1
        z = m_depth * 1
        self.br.sendTransform((x, y, z),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "otamesi",
                              "head_rgbd_sensor_link")

        return self.msgLR,x,z,h_angle
    '''	
    def human_howmany(self):
	body_estimation = Body('/home/demulab/dspl_ws/src/hsr/humanpose/src/pytorch-openpose/model/body_pose_model.pth')
	candidate, subset = body_estimation(self.oriImg)
        canvas = copy.deepcopy(self.oriImg)
        canvas,self.msgLR,xpoint,ypoint = util.draw_bodypose(canvas, candidate, subset, self.oriImg)
        m_depth = self.dep[ypoint][xpoint]
        #print(xpoint)
        #print(ypoint)

        h_angle = (xpoint - self.width/2) * (self.hfov/self.width)
        v_angle = (ypoint - self.height/2) * (self.vfov/self.height)

        x = m_depth * math.sin(h_angle) * 1
        y = m_depth * math.sin(v_angle) * 1
        z = m_depth * 1 

	return many
    '''
