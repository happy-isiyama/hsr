#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy

import numpy as np
from sklearn.decomposition import PCA

import rospy
import numpy as np
from geometry_msgs.msg import Quaternion
import tf.transformations as tft
import hsrb_interface
from hsrb_interface import geometry
import pcl
from sensor_msgs import point_cloud2
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from detectron2.config import get_cfg
from detectron2.engine import DefaultPredictor
import torch

# Set up the Detectron2 configuration and model
cfg = get_cfg()
cfg.merge_from_file("/home/demulab/detectron2/configs/COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
cfg.MODEL.WEIGHTS = "/home/demulab/dspl_ws/include/all_420k/model_final.pth"
cfg.MODEL.DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
predictor = DefaultPredictor(cfg)



def get_object_mask_from_detectron2(image, predictor, object_class):
    # Run Detectron2 prediction on the input image
    outputs = predictor(image)

    # Find the index of the bottle class in the metadata
    object_class_index = predictor.metadata.thing_classes.index(object_class)

    # Create an empty mask with the same size as the input image
    height, width, _ = image.shape
    object_mask = np.zeros((height, width), dtype=bool)

    # Iterate over the instances detected by Detectron2
    for i, class_index in enumerate(outputs["instances"].pred_classes):
        if class_index == object_class_index:
            # Get the mask for the current bottle instance and combine it with the overall bottle_mask
            instance_mask = outputs["instances"].pred_masks[i].cpu().numpy()
            object_mask |= instance_mask

    return object_mask

def process_mask(rgb_image, depth_image, camera_intrinsics, mask):
    height, width = depth_image.shape

    # Create 3D point cloud from depth image
    point_cloud = create_point_cloud(depth_image, camera_intrinsics)

    # Apply the mask to the point cloud
    masked_point_cloud = point_cloud[mask]

    # Compute the centroid and principal axes of the masked point cloud
    centroid, v1, v2, v3 = compute_centroid_and_principal_axes(masked_point_cloud)

    return masked_point_cloud, centroid, v1, v2, v3



def depth_image_callback(depth_msg):
    global depth_image, bridge
    try:
        depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
    except CvBridgeError as e:
        rospy.logerr(e)


def rgb_image_callback(rgb_msg):
    global rgb_image, bridge
    try:
        rgb_image = bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)


def generate_point_cloud(rgb_image, depth_image, camera_intrinsics):
    points = []

    fx = camera_intrinsics[0, 0]
    fy = camera_intrinsics[1, 1]
    cx = camera_intrinsics[0, 2]
    cy = camera_intrinsics[1, 2]

    for v in range(rgb_image.shape[0]):
        for u in range(rgb_image.shape[1]):
            color = rgb_image[v, u]
            Z = depth_image[v, u] / 1000.0  # Convert to meters
            if Z == 0:
                continue
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            points.append([X, Y, Z] + list(color))

    point_cloud = pc2.create_cloud_xyz32(depth_msg.header, points)
    return point_cloud


def process_rgbd_images(rgb_image, depth_image, camera_intrinsics):
    # Generate 3D point cloud
    point_cloud = generate_point_cloud(rgb_image, depth_image, camera_intrinsics)

    # Compute centroid and principal axes
    centroid, v1, v2, v3 = compute_centroid_and_axes(point_cloud)

    # Call the grasp_object_with_orientation function with the computed centroid and axes
    grasp_object_with_orientation(centroid, v1, v2, v3)


    
if __name__ == "__main__":
    rospy.init_node("process_rgbd_images_node")

    bridge = CvBridge()
    depth_image = None
    rgb_image = None
    print("Waiting for images...")
    depth_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", Image, depth_image_callback)
    print("Waiting for images...")
    rgb_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, rgb_image_callback)
    print("Waiting for images...")
    object_mask = get_object_mask_from_detectron2(rgb_image, predictor, "pocky")
    # Set Xtion camera's intrinsics matrix
    fx, fy = 528.0, 528.0  # Focal lengths in pixels
    cx, cy = 320.0, 240.0  # Optical center coordinates

    camera_intrinsics = np.array([[fx, 0, cx],
                                  [0, fy, cy],
                                  [0, 0, 1]])

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if depth_image is not None and rgb_image is not None:
            masked_point_cloud, centroid, v1, v2, v3 = process_mask(rgb_image, depth_image, camera_intrinsics, bottle_mask)
           # process_rgbd_images(rgb_image, depth_image, camera_intrinsics)
        rate.sleep()

