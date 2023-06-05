#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import copy
import math
import random
import cv2
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import subprocess
import rospy
import rospkg
import tf
from std_msgs.msg import String 
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, PointCloud2
from detectron2_ros.msg import Result
from std_srvs.srv import Empty, EmptyResponse


obj_list = ["chips_can","master_chef_can","cracker_box","sugar_box","tomato_soup_can","mustard_bottle","tuna_fish_can","pudding_box","gelation_box","potted_meat_can","banana","strawberry","apple","lemon","peach","pear","orange","plum","pitcher_base","bleach_cleaner","windex_bottle","wine_glass","bowl","mug","sponge","skillet_lid","plate","fork","spoon","knife","spatula","power_drill","wood_block","scissors","padlock","key","large_marker","small_marker","adjustable_wrench","phillips_screwdriver","flat_screwdriver","plastic_bolt","plastic_nut","hammer","small_clamp","medium_clamp","large_clamp","extra_large_clamp","mini_soccer_ball","softball","baseball","tennis_ball","racquetball","golf_ball","chain","foam_brick","dice","a_marbles","b_marbles","c_marbles","d_marbles","e_marbles","f_marbles","a_cups","b_cups","c_cups","d_cups","e_cups","f_cups","g_cups","h_cups","i_cups","j_cups","a_toy_airplane","b_toy_airplane","c_toy_airplane","d_toy_airplane","e_toy_airplane","f_toy_airplane","g_toy_airplane","h_toy_airplane","i_toy_airplane","j_toy_airplane","k_toy_airplane","a_lego_duplo","b_lego_duplo","c_lego_duplo","d_lego_duplo","e_lego_duplo","f_lego_duplo","g_lego_duplo","h_lego_duplo","i_lego_duplo","j_lego_duplo","k_lego_duplo","l_lego_duplo","m_lego_duplo","timer","rubiks_cube"]
class Icp():
    def __init__(self):
        #rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color",Image,self.img)
        rospy.Subscriber("/camera/color/image_raw",Image,self.img)
        #rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points",PointCloud2,self.point)
        rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_point",PointCloud2,self.point, queue_size = 1)
        self.ycb_tmp = "/home/demulab/dspl_ws/src/hsr/rcap_2023/meshes/ycb_tmp.pcd"

        rospy.Subscriber('/detectron2_ros/result', Result, self.maskrcnn_CB, queue_size=1)

        self.pub = rospy.Publisher("yuki",String,queue_size=10)

        self.bridge = CvBridge()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        # Mask R-CNN Callback
        self.bboxes = []
        self.class_ids = []
        self.class_names = []
        self.masks = [] 
        self.scores = []
        # PointCloud Callback
        self.pcd_x = [] 
        self.pcd_y = []
        self.pcd_z = []
        self.pcd_width = 0
        self.pcd_height = 0

        self.obj = "default"
        self.msg = String()
        rospy.Service("yuki_ishiyama",Empty,self.main)

    def img(self,msg):
        self.oriImg = self.bridge.imgmsg_to_cv2(msg)
        #print("img")

    def point(self,msg):
        #print("point")
        pcd_x = []
        pcd_y = []
        pcd_z = []
        
        for p in pc2.read_points(msg):
            pcd_x.append(p[0])
            pcd_y.append(p[1])
            pcd_z.append(p[2])

        self.pcd_x = pcd_x
        self.pcd_y = pcd_y
        self.pcd_z = pcd_z
        self.pcd_width = msg.width
        #print(self.pcd_width)
        self.pcd_height = msg.height
        #print(self.pcd_height)

    def maskrcnn_CB(self, msg):
        #print("maskrcnn_CB")
        self.class_ids = msg.class_ids
        self.class_names = msg.class_names
        self.masks = msg.masks
        self.scores = msg.scores
        self.bboxes = msg.boxes

        #print(self.class_names)
        #print(type(self.class_names))
    def main(self,req):
        try:
            #print("main")
            estimate_timer_start = rospy.get_time()
            
            index_numbers = []
            choise_object = 0  # 適当に999で初期化
            
            cut_pcd_x = []
            cut_pcd_y = []
            cut_pcd_z = []
            
            class_names = self.class_names
            class_ids = self.class_ids
            masks = self.masks
            
            pcd_x = np.array(self.pcd_x)
            pcd_y = np.array(self.pcd_y)
            pcd_z = np.array(self.pcd_z)
            #print(pcd_x.shape)
            pcd_x = pcd_x.reshape(480,640)
            pcd_y = pcd_y.reshape(480,640)
            pcd_z = pcd_z.reshape(480,640)
            #print("owari")

            ###################################################################################
            # Mask R-CNNの検出領域群から, 完全ランダムに選択
            #choise_object = random.choice(index_numbers)
            ###################################################################################
            obj_name = "cup"

            #obj_id = class_ids[choise_object]
            for i in range(len(self.class_names)):
                if self.class_names[i] in obj_name:
                    choice_object = i
                    pass
            #print(choice_object)
            #print(self.class_names[choice_object])
            mask_input_img = self.bridge.imgmsg_to_cv2(masks[choise_object], desired_encoding='8UC1')
            edge = mask_input_img
            kernel = np.ones((5, 5), np.uint8)
            #print("1")
            mask_input_img_erode = cv2.erode(mask_input_img, kernel, iterations=2)
            #print("2")
            #edge = cv2.bitwise_xor(mask_input_img, mask_input_img_erode)
            #print("3")
            conts, _ = cv2.findContours(edge, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #print("4")
            x, y, w, h = cv2.boundingRect(conts[0])
            #print("5")
            for i in range(h):
                for j in range(w):
                    if (edge[i+y][j+x] == 255
                    and np.isnan(pcd_x[i+y][j+x]) == False
                    and np.isnan(pcd_y[i+y][j+x]) == False
                    and np.isnan(pcd_z[i+y][j+x]) == False):
                        cut_pcd_x.append(pcd_x[i+y][j+x])
                        cut_pcd_y.append(pcd_y[i+y][j+x])
                        cut_pcd_z.append(pcd_z[i+y][j+x])
            
            #subscriveを書く
            for i in range(len(obj_list)):
                if obj_list[i] in obj_name:
                    self.obj = "/home/demulab/dspl_ws/src/hsr/rcap_2023/meshes/"+str(obj_list[i])+".pcd"
                    print("self.obj: ", self.obj)
                    pass
                else:
                    print("aho")
            cut_pcd_x = np.array(cut_pcd_x)
            #print(cut_pcd_x.shape)
            #print("pcd_data")
            #print(cut_pcd_x,cut_pcd_y,cut_pcd_z)
            pcd_data = self.convert_pcl(cut_pcd_x, cut_pcd_y, cut_pcd_z)

            result = self.icp(pcd_data)
            result = result.transformation
            #print("result: ",result)
            pick_position_from_camera = self.position_decision_for_id0(result)
            #print("ida")
            pick_quaternion_from_camera = self.pose_decision_for_id0(result)
            #print("ishiyama")
            time_name = rospy.get_time()
            self.regist_work(pick_position_from_camera, pick_quaternion_from_camera, time_name)
            #print("kawasumi")
            self.bboxes = []
            self.class_ids = []
            self.class_names = []
            self.masks = []
            self.scores = []
            self.pcd_x = []
            self.pcd_y = []
            self.pcd_z = []
            self.pcd_width = 0
            self.pcd_height = 0
            pick_position_from_camera = []
            pick_quaternion_from_camera = []
            pick_position_from_world = []
            pick_quaternion_from_world = []

        except (IndexError, ValueError) as e:
            flag = Bool()
            flag.data = True
            rospy.sleep(0.2)
            #self.pass_pub.publish(flag)
            rospy.logwarn(e)
        return EmptyResponse()

    def position_decision_for_id0(self, result):
        """
        @brief Mask R-CNNの認識クラスが0のときの把持位置推定
               ICP結果の回転行列から算出
        @param (result) ICPから得られた回転行列
        @return (position_x, position_y, position_z) 把持位置のx, y, z座標
        
        """
        position_x = result[0][3]
        position_y = result[1][3]
        position_z = result[2][3]
        #print(position_x,position_y,position_z)
        return [position_x, position_y, position_z]

    def pose_decision_for_id0(self, result):
        """
        @brief Mask R-CNNの認識クラスが0のときの把持姿勢推定
               ICP結果の回転行列から算出
        @param (result) ICPから得られた回転行列
        @return (quaternion) 把持姿勢(クォータニオン形式)
        
        """
        roll  = np.arctan2(result[2][1], result[2][2])
        pitch = np.arctan2(-1*result[2][0], np.sqrt(result[2][1]*result[2][1] + result[2][2]*result[2][2]))
        yaw   = np.arctan2(result[1][0], result[0][0])
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        #print(quaternion)
        return quaternion

    def regist_work(self, position, quaternion, t_name):
        """
        @brief カメラ座標系からみたワークの位置姿勢をTFに登録
        @param (position) 位置[x, y, z]
        @param (quaternion) 姿勢[x, y, z, w]
        @param (t_name) 任意の時間(登録座標系名をその都度変更したいため)
        
        """
        self.tf_broadcaster.sendTransform(position,                # ベースから見た直交座標位置
                                          quaternion,              # ベースから見た姿勢(クォータニオン)
                                          rospy.Time.now(),        # タイムスタンプ (default: rospy.Time.now())
                                          "ida_kamo", # 登録したい座標系の任意名
                                          "/camera_depth_optical_frame",
                                          #'head_rgbd_sensor_rgb_frame',   # ベースとなる座標系
                                          )

    def verify_pcd(self, pcd_x, pcd_y, pcd_z):
        """
        @brief 点群をmatplotlibで３次元描画
        @param (pcd_x) 点群のx成分の配列
        @param (pcd_y) 点群のy成分の配列
        @param (pcd_z) 点群のz成分の配列
        
        """
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.plot(pcd_x, pcd_y, pcd_z, 'o', ms=4, mew=0.5)
        plt.show()

    def convert_pcl(self, pcd_x, pcd_y, pcd_z):
        header = '''# .PCD v0.7 - Point Cloud Data file format
        VERSION 0.7
        FIELDS x y z
        SIZE 4 4 4
        TYPE F F F
        COUNT 1 1 1
        WIDTH %d
        HEIGHT %d
        VIEWPOINT 0 0 0 1 0 0 0
        POINTS %d
        DATA ascii
        '''
        with open(self.ycb_tmp, 'w') as f:
            cloud_list = []
            f.write(header % (len(pcd_x), 1, len(pcd_x)))
            f.write('\n')
            for p in range(len(pcd_x)):
                f.write('%f %f %f' % (pcd_x[p], pcd_y[p], pcd_z[p]))
                f.write('\n')
            for p in range(len(pcd_x)):
                cloud_list.append(pcd_x[p])
                cloud_list.append(pcd_y[p])
                cloud_list.append(pcd_z[p])
            f.write("\n")
        pcd = o3d.io.read_point_cloud(self.ycb_tmp)

        #o3d.visualization.draw_geometries([pcd])
        #print("pcd", pcd)
        return pcd

    def icp(self, pcd_data):
        source_down, target_down, source_fpfh, target_fpfh, voxel_size = self.prepare_dataset(pcd_data)
        result_ransac = self.execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
        result_icp = self.refine_registration(source_down, target_down, result_ransac, source_fpfh, target_fpfh, voxel_size)
        #print(result_icp)
        return result_icp

    def prepare_dataset(self, pcd_data):
        source = o3d.io.read_point_cloud(self.obj)
        #print("source: ",source)
        target = pcd_data
        
        # set 1/10 of the target pointcloud as the base size
        voxel_size = np.abs((target.get_max_bound() - target.get_min_bound())).max() / 500
        
        #source_down = o3d.geometry.voxel_down_sample(source, voxel_size) # old
        source_down = source.voxel_down_sample(voxel_size)
        #target_down = o3d.geometry.voxel_down_sample(target, voxel_size)
        target_down = target.voxel_down_sample(voxel_size)
        #o3d.visualization.draw_geometries([source_down])

        #o3d.geometry.estimate_normals(source_down, o3d.geometry.KDTreeSearchParamHybrid(radius = voxel_size, max_nn=30))
        source_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius = voxel_size, max_nn=20))#法線ベクトル
        #print("down_sampled: ", source_down)
        #o3d.geometry.estimate_normals(target_down, o3d.geometry.KDTreeSearchParamHybrid(radius = voxel_size, max_nn=30))
        target_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius = voxel_size, max_nn=20))#法線ベクトル
        source_fpfh = o3d.registration.compute_fpfh_feature(source_down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size, max_nn=30))#特徴量を計算
        #print("source_fpfh:",source_fpfh)
        target_fpfh = o3d.registration.compute_fpfh_feature(target_down, o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size, max_nn=30))#特徴量を計算
        #print("target_fpfh:",target_fpfh)
        return source_down, target_down, source_fpfh, target_fpfh, voxel_size

    def execute_global_registration(self, source_down, target_down, source_fpfh, target_fpfh, voxel_size):
        distance_threshold = voxel_size
        result = o3d.registration.registration_ransac_based_on_feature_matching(source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
                                                                                o3d.registration.TransformationEstimationPointToPoint(), 4,
                                                                                [o3d.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
                                                                                o3d.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
                                                                                o3d.registration.RANSACConvergenceCriteria(max_iteration=400000,
                                                                                                                      max_validation=5000))
        return result

    def refine_registration(self, source, target, result_ransac, source_fpfh, target_fpfh, voxel_size):
        est_ptpln = o3d.registration.TransformationEstimationPointToPlane()
        criteria = o3d.registration.ICPConvergenceCriteria(max_iteration=50)
        distance_threshold = voxel_size
        result = o3d.registration.registration_icp(source, target, distance_threshold, result_ransac.transformation, est_ptpln, criteria)
        correspondence_set = result.correspondence_set
        correspondence_set = np.asarray(correspondence_set)
        #print("a")
        #o3d.visualization.draw_geometries([pcd])
        #mach = np.array_equal(correspondence_set,np.array([]))
        #pu = rospy.Publisher("a",String,queue_size=10)
        if correspondence_set.size == 0:
            print("もう一回")
            #Icp()
            #rospy.sleep(0.1)
            #rospy.ServiceProxy("yuki_ishiyama",Empty)
            self.msg.data = "False"
            self.pub.publish(self.msg)
            Icp()
            print("end")
        else:
            print(result)
            self.msg.data = "True"
            self.pub.publish(self.msg)
            Icp()
            return result
        '''
        print(mach)
        if mach == "True":
            self.main()
            print("もう一回")
            rospy.ServiceProxy("yuki_ishiyama",Empty)
        else:
            return result
        '''
        #return result

    def rpy_limit(self, quaternion, obj_id):
        rpy = tf.transformations.euler_from_quaternion(quaternion)
        rpy = list(rpy)
        rad_limit = np.deg2rad(25)
        if obj_id == 0:
            for i in range(2):
                if -rad_limit < rpy[i] and rpy[i] < rad_limit:
                    rpy[i] = rpy[i]
                elif rpy[i] < -rad_limit:
                    rpy[i] = -rad_limit
                elif rpy[i] > rad_limit:
                    rpy[i] = rad_limit
        return tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])

if __name__ == '__main__':
    try:
        rospy.init_node('Icp_match', anonymous=True)
        Icp()
        print("icp match")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
