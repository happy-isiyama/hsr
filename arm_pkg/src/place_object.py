#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Quaternion

# HSRの制御クラスのインスタンスを作成
robot = moveit_commander.RobotCommander()
arm = moveit_commander.MoveGroupCommander("arm_group")
gripper = moveit_commander.MoveGroupCommander("gripper")

# 机にオブジェクトを置く関数
def place_object():
    # 手先を下向きにするために姿勢を変更する
    arm.set_named_target("up")
    arm.go()

    # 机の位置を決定する
    table_height = 0.8
    table_x = 0.6
    table_y = 0.0
    table_z = table_height + 0.01  # オブジェクトを置く位置を少し上に設定する

    # 机の上にオブジェクトを置くための目標姿勢を定義する
    place_pose = PoseStamped()
    place_pose.header.frame_id = "base_footprint"
    place_pose.pose.position.x = table_x
    place_pose.pose.position.y = table_y
    place_pose.pose.position.z = table_z
    place_pose.pose.orientation = Quaternion(0, 0, 0, 1)  # 手先を下向きにするためにクォータニオンはデフォルトでよい

    # オブジェクトを置くための移動計画を作成する
    arm.set_pose_target(place_pose)
    plan = arm.plan()
    
    # HSRを目標姿勢まで動かす
    arm.execute(plan)

    # オブジェクトを放す
    gripper.set_joint_value_target([0.8])
    gripper.go()
