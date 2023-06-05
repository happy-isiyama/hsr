#!/usr/bin/python3
# -*- coding: utf-8 -*-

import hsrb_interface
from hsrb_interface import Robot
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from hsrb_interface import geometry
import sys
import tf2_ros
import tf
import tf2_geometry_msgs
from tf2_geometry_msgs import PointStamped as TF2PointStamped

from std_msgs.msg import String
import math
# 移動のタイムアウト[s]
_MOVE_TIMEOUT=60.0
# 把持力[N]
_GRASP_FORCE=0.5
# ボトルのtf名
#_BOTTLE_TF='ar_marker/201'
# グリッパのtf名
_HAND_TF='hand_palm_link'

# ロボット機能を使うための準備
robot = hsrb_interface.Robot()
omni_base = robot.get('omni_base')
whole_body = robot.get('whole_body')
collision_world = robot.try_get('global_collision_world')
gripper = robot.get('gripper')
tts = robot.get('default_tts')

class Arm():
    def __init__(self):
        self.listener = tf.TransformListener()
        self.appear_list = ''
        rospy.Subscriber("/appear_list", String, self.apearCB)
    
    def apearCB(self,msg):
        self.appear_list = msg.data

    def take_list(self):
        object_list = []
        self.appear_list = ''
        rospy.sleep(2.0)
        if self.appear_list == '' or self.appear_list == '':
            rospy.loginfo("No object")
            return 'no_object'
        else:
            split_string = self.appear_list.split(",")  # カンマで分割してリスト化
            #2重配列でオブジェクトと距離を格納する
            for i in range(len(split_string)):
                if i % 2 == 0:
                    object_list.append([split_string[i], float(split_string[i + 1])])
            return object_list

    def shelf_collision(self, object_list,pick_object,x,y,z):
        #print("add_collision")
        #print(omni_base.pose)
        collision_world.remove_all()
        box_size = (x, y, z)
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        for obj in object_list:
            if obj[0] == pick_object:
                pass
            else:
         #       print(obj[0])
                obj_name = obj[0]  # オブジェクト名
                obj_frame_id = obj_name  # オブジェクトのframe_id
                try:
                    transform = tf_buffer.lookup_transform(obj_frame_id, 'map', rospy.Time(), rospy.Duration(6.0))

                    # カメラのフレームID（通常は "head_rgbd_sensor_rgb_frame"）
                    camera_frame_id = "head_rgbd_sensor_rgb_frame"
                    # カメラフレームからmapフレームまでの変換を取得する
                    camera_to_map_transform = tf_buffer.lookup_transform("map", camera_frame_id, rospy.Time(0), rospy.Duration(1.0))

                    # TransformStampedからPoseStampedに変換
                    object_pose = PoseStamped()
                    object_pose.header = transform.header
                    object_pose.pose.position = transform.transform.translation
                    object_pose.pose.orientation = transform.transform.rotation

                    # カメラの位置情報をPoseStampedに変換
                    camera_pose = PoseStamped()
                    camera_pose.header = camera_to_map_transform.header
                    camera_pose.pose.position = camera_to_map_transform.transform.translation
                    camera_pose.pose.orientation = camera_to_map_transform.transform.rotation
          #          print(object_pose)
          #          print(camera_pose)

                    # オブジェクトの位置をカメラから見た位置に変換
                    object_pose_camera = tf2_geometry_msgs.do_transform_pose(object_pose, camera_to_map_transform)
           #         print(object_pose_camera)
                    position_tuple = (-object_pose_camera.pose.position.x+omni_base.pose[0],
                                  -object_pose_camera.pose.position.y+omni_base.pose[1],
                                  object_pose.pose.position.y)


            #        print(position_tuple)
                    orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, 0)

                    pose_tuple = (position_tuple, orientation_tuple)

                    # ボックスを追加
                    collision_world.add_box(x=box_size[0], y=box_size[1], z=box_size[2], pose=pose_tuple, frame_id='map', name=obj_name + "static")
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logwarn("Failed to get transform from {} to map".format(obj_frame_id))

            omni_base.collision_world = collision_world
            whole_body.collision_world = collision_world
          #  print(whole_body.collision_world)


    def add_collision(self, object_list,x,y,z):
        #print("add_collision")
        #print(omni_base.pose)
        collision_world.remove_all()

        box_size = (x, y, z)
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        for obj in object_list:
         #   print(obj[0])
            obj_name = obj[0]  # オブジェクト名
            obj_frame_id = obj_name  # オブジェクトのframe_id
            try:
                transform = tf_buffer.lookup_transform(obj_frame_id, 'map', rospy.Time(), rospy.Duration(6.0))

                # カメラのフレームID（通常は "head_rgbd_sensor_rgb_frame"）
                camera_frame_id = "head_rgbd_sensor_rgb_frame"
                # カメラフレームからmapフレームまでの変換を取得する
                camera_to_map_transform = tf_buffer.lookup_transform("map", camera_frame_id, rospy.Time(0), rospy.Duration(1.0))

                # TransformStampedからPoseStampedに変換
                object_pose = PoseStamped()
                object_pose.header = transform.header
                object_pose.pose.position = transform.transform.translation
                object_pose.pose.orientation = transform.transform.rotation

                # カメラの位置情報をPoseStampedに変換
                camera_pose = PoseStamped()
                camera_pose.header = camera_to_map_transform.header
                camera_pose.pose.position = camera_to_map_transform.transform.translation
                camera_pose.pose.orientation = camera_to_map_transform.transform.rotation
          #      print(object_pose)
          #      print(camera_pose)

                # オブジェクトの位置をカメラから見た位置に変換
                object_pose_camera = tf2_geometry_msgs.do_transform_pose(object_pose, camera_to_map_transform)
           #     print(object_pose_camera)
                position_tuple = (-object_pose_camera.pose.position.x+omni_base.pose[0], 
                                  -object_pose_camera.pose.position.y+omni_base.pose[1], 
                                  object_pose.pose.position.y)


            #    print(position_tuple)
                orientation_tuple = tf.transformations.quaternion_from_euler(0, 0, 0)

                pose_tuple = (position_tuple, orientation_tuple)

                # ボックスを追加
                collision_world.add_box(x=box_size[0], y=box_size[1], z=box_size[2], pose=pose_tuple, frame_id='map', name=obj_name + "static")
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Failed to get transform from {} to map".format(obj_frame_id))

        omni_base.collision_world = collision_world
        whole_body.collision_world = collision_world
        #print(whole_body.collision_world)

    def ar_pick(self,ar):
        topic_tf = 'ar_marker/' + ar
        object_to_hand = geometry.pose(z=0, ek=-1.57)
        
        # handを0.1[m]上に移動させる姿勢
        hand_up = geometry.pose(x=0.1)
        # handを0.5[m]手前に移動させ:
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

#床に落ちているオブジェクトを把持する。　手先は下に向けた状態で把持する。
    def instance_floor(self,object_name):
        object_to_front = geometry.pose(z=-0.1, ek=-1.57)
        object_to_push = geometry.pose(z=0.08)
        object_to_up = geometry.pose(x=0.1)
        object_to_back = geometry.pose(z=-0.5)
        object_to_hand = geometry.pose(y=-0.1, ek=-1.57, ei=3.14, ej=1.57)
        whole_body.linear_weight = 30

        topic_tf = object_name
        whole_body.loking_hand_constraint = True
        gripper.command(1.0)
       # whole_body.move_to_go()
        rospy.sleep(2.0)
        #首を下に向ける
        whole_body.move_to_joint_positions({'head_tilt_joint': -0.8})
        rospy.sleep(2.0)
        if object_name in ['cleaner','cracker_box','cleanser']:
            #横から掴みに行く
            whole_body.move_end_effector_pose(object_to_front, topic_tf)
            rospy.sleep(2.0)
            #手先を相対位置0.1[m]下に移動
            whole_body.move_end_effector_by_line((0, 0, 1), 0.1)
            rospy.sleep(2.0)
            # 力を指定して把持する
            gripper.apply_force(_GRASP_FORCE)
            # シミュレータのgrasp hackのための待ち時間。実機では不要
            rospy.sleep(2.0)
            # 手先を上に移動。昇降を使う
            whole_body.move_end_effector_pose(object_to_up, _HAND_TF)
            rospy.sleep(2.0)
            #元の姿勢に戻る
            #whole_body.move_to_neutral()



        else:
       # whole_body.gaze_point(point=geometry.Vector3(x=1.0,y=0,z=0.5), ref_frame_id='base_link')
            whole_body.move_end_effector_pose(object_to_hand, topic_tf)
            rospy.sleep(2.0)
            #whole_body.move_to_joint_positions({'wrist_roll_joint':0 })
            rospy.sleep(2.0)
            #手先を相対位置0.1[m]下に移動 
            whole_body.move_end_effector_by_line((0, 0, 1), 0.06)
            rospy.sleep(2.0)
            # 力を指定して把持する
            gripper.apply_force(_GRASP_FORCE)
            # シミュレータのgrasp hackのための待ち時間。実機では不要
            rospy.sleep(2.0)
            # 手先を上に移動。昇降を使う
            whole_body.move_to_joint_positions({'arm_lift_joint': 0.2})

            whole_body.move_to_neutral()
        sys.exit()


    def place_object(self):
        whole_body.looking_hand_constraint = False
        whole_body.move_to_neutral()
        rospy.sleep(1.0)
        whole_body.move_to_joint_positions({'arm_lift_joint': 0.4})
        rospy.sleep(1.0)
        whole_body.linear_weight = 100
        rospy.sleep(1.0)
        whole_body.move_end_effector_by_line((0, 0, 1), 0.2)
        #omni_base.go_abs(0,0,0)
        rospy.sleep(1.0)
        whole_body.move_end_effector_by_line((0, 0, 1), 0.2)
        #whole_body.linear_weight = 1.0
        rospy.sleep(1.0)
        #omni_base.go_abs(0,0,0)
        print(omni_base.pose)
        #0.5m前に移動
        omni_base.go_rel(0.2, 0, 0.0, 300.0)
        rospy.sleep(2.0)
        whole_body.move_to_joint_positions({'arm_lift_joint': 0.25})
        rospy.sleep(2.0)
        gripper.command(1.0)
        omni_base.go_rel(-0.2, 0, 0.0, 300.0)
        whole_body.move_to_neutral()

    def give_object(self):
        #手先を相対位置0.1[m]下に移動
        whole_body.move_end_effector_by_line((0, 0, 0.3), 0.1)
        rospy.sleep(2.0)
        gripper.command(1.0)
        rospy.sleep(2.0)

    def place_object2(self):
        whole_body.looking_hand_constraint = False
        whole_body.move_to_neutral()
        rospy.sleep(2.0)
        #手首を上に向け
        whole_body.move_to_joint_positions({'wrist_flex_joint': 0.0})
        rospy.sleep(2.0)
        whole_body.move_to_joint_positions({'arm_lift_joint': 0.69})
        rospy.sleep(2.0)
        whole_body.move_to_joint_positions({'arm_flex_joint': -1.57})
        gripper.command(1.0)

    def instance_desk(self,object_name):
        topic_tf = object_name
        object_to_hand = geometry.pose(z=-0.1, ek=-1.57)

        # handを0.1[m]上に移動させる姿勢
        hand_push = geometry.pose(z=0.1)
        hand_up = geometry.pose(x=0.1)
        # handを0.5[m]手前に移動させる姿勢
        hand_back = geometry.pose(z=-0.5)

        whole_body.lookking = False


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
        object_to_hand = geometry.pose(z=-0.15, ek=-1.57)

        # handを0.1[m]上に移動させる姿勢
        hand_push = geometry.pose(z=0.15)
        hand_up = geometry.pose(x=0.1)
        # handを0.5[m]手前に移動させる姿勢
        hand_back = geometry.pose(z=-0.5)

        gripper.command(0.7)
        whole_body.move_to_neutral()
        whole_body.looking_hand_constraint = False
        #whole_body.move_to_go()
        #whole_body.move_to_joint_positions({'arm_lift_joint': 0.3})
        whole_body.looking_hand_constraint = True
        #whole_body.gaze_point(point=geometry.Vector3(x=1.0,y=0,z=0.5), ref_frame_id='base_link')
        self.listener.waitForTransform("head_rgbd_sensor_rgb_frame", topic_tf, rospy.Time(), rospy.Duration(4.0))
        #rospy.sleep(3.0)
        whole_body.move_end_effector_pose(object_to_hand, topic_tf)
        rospy.sleep(2.0)
        #gripper.command(1.0)
        # 力を指定して把持する
        whole_body.move_end_effector_pose(hand_push, _HAND_TF)
        rospy.sleep(2.0)
        gripper.apply_force(_GRASP_FORCE)
        # シミュレータのgrasp hackのための待ち時間。実機では不要
        rospy.sleep(2.0)
        # 手先相対で上にハンドを移動
        whole_body.move_end_effector_pose(hand_up, _HAND_TF)
        rospy.sleep(2.0)
        # 手先相対で後ろにハンドを移動
        whole_body.move_end_effector_pose(hand_back, _HAND_TF)
        rospy.sleep(2.0)
        # 初期姿勢に遷移
        whole_body.move_to_neutral()
        tts.say('把持に成功しました')
        gripper.command(1.0)
        sys.exit()


    

if __name__=='__main__':
    arm = Arm()
    rospy.sleep(5.0)
    #首を下に向ける
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
        #相対座標で1.5m前に移動
        #try:
            #whole_body.angular_weight = 80.0
            #whole_body.move_end_effector_pose(geometry.pose(x=3.0, z=0.7) ,ref_frame_id='map')
        #arm.instance_shelf(pick_object)
        #rospy.sleep(1.0)
        #except rospy.ROSInterrup:
        #    print("error")
        #    pass
            # omni_base.go_rel(3.0,0.12,0.043,100.0)
        #rospy.sleep(1.0)
    #手先を下に向けながらオブジェクトを置く
    #whole_body.move_end_effector_pose(geometry.pose(x=0.2, y=0.0, z=0.5, ek=1.57), ref_frame_id='base_link')だと手先が上を向いているので下に向けたい
    #mein place object
    #arm.place_object2(0.2,0.0,0.5,0,-math.pi/2,0)
    #whole_body.gaze_point(point=geometry.Vector3(x=1.0,y=0,z=0.5), ref_frame_id='base_link')
    #omni_base.go_pose(geometry.pose(z=-0.5, ei=3.14, ej=-1.57), 100.0, ref_frame_id='bottle')
    #arm.grasp_object("bottle")
    #collision_world.remove_all()

    #現在の座標を取得
"""
    pick_point = omni_base.pose
    print(pick_point)
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
            omni_base.go_abs(pick_point[0],pick_point[1],pick_point[2],100.0)
            gripper.command(1.0)
            rospy.sleep(3.0)
"""
    #arm.instance_shelf("bottle")
   # arm.instance_floor("bottle")
