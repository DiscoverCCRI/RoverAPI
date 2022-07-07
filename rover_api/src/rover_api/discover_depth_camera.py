#!/usr/bin/env python3

from datetime import datetime
from os.path import exists
from os import mkdir
from sensor_msgs.msg import Image, PointCloud2
from rospy import sleep, Subscriber, init_node, loginfo
import cv2
from cv_bridge import CvBridge


class DepthCamera:
    def __init__(self):
        try:
            init_node("discover_rover")
            loginfo("Depth Camera initialized!")
        finally:
            self._depth_buffer = []
            self._pc_buffer = []
            self.__subscribe_to_depth_image()
            self.__subscribe_to_point_cloud()

            if not exists("/root/photos/"):
                mkdir("/root/photos/")

            # allows the buffer to store an entire image before init is over
            sleep(1)

    def __subscribe_to_depth_image(self):
        Subscriber("/camera/depth/image_rect_raw", Image,
                   self.__callback_get_depth)

    def __subscribe_to_point_cloud(self):
        Subscriber("/camera/depth/color/points", PointCloud2,
                   self.__callback_get_point_cloud)

    def __callback_get_depth(self, message: Image):
        if len(self._depth_buffer) >= 15:
            self._depth_buffer.pop(0)

        self._depth_buffer.append(message)

    def __callback_get_point_cloud(self, message: PointCloud2):
        if len(self._pc_buffer) >= 15:
            self._pc_buffer.pop(0)

        self._pc_buffer.append(message)

    def take_depth_photo(self):
        bridge = CvBridge()

        # get the time object
        depth_img = self._depth_buffer[-1]

        # convert to an OpenCV image
        img = bridge.imgmsg_to_cv2(depth_img, desired_encoding="16UC1")

        # convert to a python datetime object
        py_time = datetime.fromtimestamp(depth_img.header.stamp.to_time())

        # convert object to string
        time_str = py_time.strftime("%d-%m-%Y_%H:%M:%S")
        img_str = "/root/photos/leo_depth_" + time_str + ".jpg"

        cv2.imwrite(img_str, img)

    def get_point_cloud(self) -> PointCloud2:
        return self._pc_buffer[-1]
