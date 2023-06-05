#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Import the required libraries
import rospy
from sensor_msgs.msg import PointCloud2
import pcl
import pcl.pcl_visualization
from sensor_msgs import point_cloud2
import pcl
import pcl.pcl_visualization
from sensor_msgs import point_cloud2
import numpy as np
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped, Quaternion
import tf.transformations as tft
import hsrb_interface
from hsrb_interface import Robot, geometry
import tf2_ros
from geometry_msgs.msg import TransformStamped


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

def point_cloud_callback(msg):
    global point_cloud_data
    point_cloud_data = msg

def find_empty_space(point_cloud_data):
    # Convert the ROS PointCloud2 message to a PCL PointCloud
    pcl_data = pcl.PointCloud()
    pcl_data.from_list(list(point_cloud2.read_points(point_cloud_data, field_names=("x", "y", "z"), skip_nans=True)))

    # Process the point cloud data to find empty spaces in the shelf (you may need to fine-tune these values)
    passthrough_filter = pcl_data.make_passthrough_filter()
    passthrough_filter.set_filter_field_name("z")
    passthrough_filter.set_filter_limits(0.5, 1.5)
    filtered_data = passthrough_filter.filter()

    # Cluster extraction to find the individual objects in the shelf
    tree = filtered_data.make_kdtree()
    ec = filtered_data.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(25000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # Iterate through the clusters to find an empty space (you may need to fine-tune these values)
    for j, indices in enumerate(cluster_indices):
        points = []
        for i in indices:
            points.append(filtered_data[i])
        cluster_data = pcl.PointCloud()
        cluster_data.from_list(points)

        # Compute the centroid of the cluster
        centroid = np.mean(cluster_data.to_array(), axis=0)[:3]

        # Check if the centroid of the cluster is in an empty space in the shelf (you may need to fine-tune these values)
        if centroid[1] > -0.5 and centroid[1] < 0.5:
            return centroid

    return pcl_data

def convert_to_point_stamped(x, y, z, frame_id='head_rgbd_sensor_rgb_frame'):
    point = PointStamped()
    point.header.frame_id = frame_id
    point.header.stamp = rospy.Time.now()
    point.point.x = x
    point.point.y = y
    point.point.z = z
    return point

def broadcast_static_tf(x, y, z, frame_id='base_link', child_frame_id='target_frame'):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transform = TransformStamped()

    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = frame_id
    static_transform.child_frame_id = child_frame_id

    static_transform.transform.translation.x = x
    static_transform.transform.translation.y = y
    static_transform.transform.translation.z = z

    static_transform.transform.rotation.x = 0
    static_transform.transform.rotation.y = 0
    static_transform.transform.rotation.z = 0
    static_transform.transform.rotation.w = 1

    broadcaster.sendTransform(static_transform)


def move_arm_to_point(point):
    # Move the arm to the target point
    whole_body.move_end_effector_pose(geometry.pose(x=point.point.x, y=point.point.y, z=point.point.z), _HAND_TF)
if __name__ == "__main__":
    rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2, point_cloud_callback)
    point_cloud_data = None

    # Wait for point cloud data to arrive
    while point_cloud_data is None and not rospy.is_shutdown():
        print("Waiting for point cloud data...")
        rospy.sleep(1)

    # Find an empty space in the shelf
    centroid = find_empty_space(point_cloud_data)

    # Print the centroid of the empty space

    print(centroid)

    # Convert the centroid to a PointStamped message
    point = convert_to_point_stamped(centroid[0], centroid[1], centroid[2])
    print(point)
    # Move the arm to the target point
    #broadcast_static_tf(point.point.x, point.point.y, point.point.z)
    move_arm_to_point(point)

