#!/home/user/anaconda3/envs/PoseCNN/bin/python
import os
import sys
from skimage import io, transform
import torch
import torchvision
from torch.autograd import Variable
from torchvision import transforms
import torch.nn.functional as F
import torch.nn as nn

import numpy as np
from PIL import Image as PIL_Image
import glob
import cv2

import rospy
from cv_bridge import CvBridge
from vision_msgs.msg import SegResult
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Empty, String, Bool, Header, Float64, Int8
from sensor_msgs.msg import CompressedImage, Image, JointState
from sensor_msgs.msg import PointCloud2, PointField
import message_filters


class SegModule:    # U2Net + HSNet
    def __init__(self):
        
        self.br = CvBridge()
        self.image_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        self.depth_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10)
        self.ts.registerCallback(self.RGBD_sub_callback)


        self.target_id_sub = rospy.Subscriber('/target_id', Int8, self.target_id_callback)
        self.target_id = -1



    def target_id_callback(self, msg):
        self.target_id = msg.data
        print("target id : ", msg.data)


    def RGBD_sub_callback(self, image_msg, depth_msg):
        # # image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1).copy()
        # # print(image.shape)
        # if self.target_id > 0:
        image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1).copy()
        depth = np.frombuffer(depth_msg.data, dtype=np.uint16).reshape(depth_msg.height, depth_msg.width, -1).copy()

        img = image.copy()
        ori_img = image.copy()

        # print(image.shape)
        print(depth[depth > 0])
        # cv2.imshow("depth", depth)
        # cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node('SegModule', anonymous=False)
    n = SegModule()

    rospy.spin()
