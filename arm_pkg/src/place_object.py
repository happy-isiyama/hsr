#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
from detectron2_ros.msg import Result
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sklearn.decomposition import PCA
import tf2_ros
from geometry_msgs.msg import TransformStamped


object_mask = None
choice_object = "pocky"
bridge = CvBridge()
def compute_centroid_and_principal_axes(point_cloud):
    centroid = np.mean(point_cloud, axis=0)
    pca = PCA(n_components=3)
    pca.fit(point_cloud)
    v1, v2, v3 = pca.components_
    return centroid, v1, v2, v3

def create_point_cloud(depth_image, camera_intrinsics):
    height, width = depth_image.shape
    fx = camera_intrinsics[0, 0]
    fy = camera_intrinsics[1, 1]
    cx = camera_intrinsics[0, 2]
    cy = camera_intrinsics[1, 2]

    u, v = np.meshgrid(np.arange(width), np.arange(height))
    u, v = u.astype(np.float32), v.astype(np.float32)

    Z = depth_image / 1000.0
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

    point_cloud = np.stack([X, Y, Z], axis=-1)
    return point_cloud


def process_mask(rgb_image, depth_image, camera_intrinsics, mask):
    height, width = depth_image.shape

    # Create a point cloud from the depth image
    point_cloud = create_point_cloud(depth_image, camera_intrinsics)

    # Apply the mask to the point cloud
    masked_point_cloud = point_cloud[mask]

    if masked_point_cloud.size == 0:
        rospy.logwarn("No points in masked point cloud. Skipping this iteration.")
        return None, None, None, None, None

    # Flatten the masked_point_cloud to a 2D array
    masked_point_cloud = masked_point_cloud.reshape(-1, 3)

    # Compute the centroid and principal axes of the masked point cloud
    centroid, v1, v2, v3 = compute_centroid_and_principal_axes(masked_point_cloud)

    return masked_point_cloud, centroid, v1, v2, v3


def detectron2_ros_callback(result):
    global object_mask
    global choice_object
    try:
        object_index = result.class_names.index(choice_object)
        object_mask = bridge.imgmsg_to_cv2(result.masks[object_index], desired_encoding='8UC1')
        print("Object found in image.")
        print(object_mask)
    except ValueError:
        rospy.logwarn("Object not found in image. Skipping this iteration.")


def rgb_image_callback(rgb_image_msg):
    global rgb_image
    rgb_image = CvBridge().imgmsg_to_cv2(rgb_image_msg, "bgr8")

def depth_image_callback(depth_image_msg):
    global depth_image
    depth_image = CvBridge().imgmsg_to_cv2(depth_image_msg, "32FC1")

def camera_info_callback(camera_info_msg):
    global camera_intrinsics
    camera_intrinsics = np.array(camera_info_msg.K).reshape(3, 3)

def object_pose():
    global rgb_image, depth_image, object_mask, camera_intrinsics

    rospy.init_node('object_pose_node', anonymous=True)

    rospy.Subscriber('/detectron2_ros/result', Result, detectron2_ros_callback)
    rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/image_raw', Image, rgb_image_callback)
    rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_raw', Image, depth_image_callback)
    rospy.Subscriber('/hsrb/head_rgbd_sensor/rgb/camera_info', CameraInfo, camera_info_callback)

    rospy.sleep(1)

    masked_point_cloud, centroid, v1, v2, v3 = process_mask(rgb_image, depth_image, camera_intrinsics, object_mask)

    rotation_angle = np.arctan2(v1[1], v1[0])
    q = quaternion_from_euler(0, 0, rotation_angle)

    object_pose_msg = PoseStamped()
    object_pose_msg.header.stamp = rospy.Time.now()
    object_pose_msg.header.frame_id = 'base_link'
    object_pose_msg.pose.position.x = centroid[0]
    object_pose_msg.pose.position.y = centroid[1]
    object_pose_msg.pose.position.z = centroid[2]
    object_pose_msg.pose.orientation.x = q[0]
    object_pose_msg.pose.orientation.y = q[1]
    object_pose_msg.pose.orientation.z = q[2]
    object_pose_msg.pose.orientation.w = q[3]

    print("Object pose:", object_pose_msg)
        # Create a StaticTransformBroadcaster

    # Convert the quaternion to Euler angles
    euler_angles = euler_from_quaternion([
        object_pose_msg.pose.orientation.x,
        object_pose_msg.pose.orientation.y,
        object_pose_msg.pose.orientation.z,
        object_pose_msg.pose.orientation.w
    ])
    ei, ej, ek = euler_angles
    # Print the Euler angles
    print("Euler angles:", ei, ej, ek)
    return ei, ej, ek
if __name__ == '__main__':
    object_pose()
