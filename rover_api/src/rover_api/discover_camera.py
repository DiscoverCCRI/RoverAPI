#!/usr/bin/env python3

from datetime import datetime
from os.path import exists
from os import mkdir
from sensor_msgs.msg import Image
from rospy import init_node, Subscriber, sleep, loginfo
import cv2
from cv_bridge import CvBridge


class Camera:
    """
    A class used to instantiate and use the raspicam on the LeoRover
    ...
    Attributes:
    -----------
    img_buffer:
        A list that contains the data of each image sent over the
        /camera/image_raw/compressed topic. Each image is stored in an array of
        unsigned 8-bit integers
    Methods:
    --------
    take_photo():
        Creates an OpenCV image from the latest image in the buffer
    __subscribe_to_image_topic():
        Creates a subscriber to subscribe to the /camera/image_raw
        topic created by the raspicam node. Then continues to run
        the function until the script is stopped
    __callback_get_image(message: Image):
        Gets the message from the /camera/image_raw topic and stores
        the image data to the image buffer
    """

    def __init__(self):
        try:
            init_node("discover_rover")
            loginfo("Camera initialized!")
        finally:
            self._img_buffer = []
            self.__subscribe_to_image_topic()

            if not exists("/root/photos/"):
                mkdir("/root/photos/")

            # allows the buffer to store an entire image before init is over
            sleep(1)

    def __subscribe_to_image_topic(self):
        Subscriber("/camera/image_raw", Image, self.__callback_get_image)

    def __callback_get_image(self, message: Image):
        time = message.header.stamp

        if len(self._img_buffer) >= 30:
            self._img_buffer.pop(0)

        # add a tuple containing the unsigned 8-bit integer data and time
        self._img_buffer.append((message, time))

    def take_photo(self):
        bridge = CvBridge()

        # get the time object
        img_tuple = self._img_buffer[-1]

        # convert to an OpenCV image
        img = bridge.imgmsg_to_cv2(img_tuple[0], 
                                   desired_encoding='passthrough')

        # convert to a python datetime object
        py_time = datetime.fromtimestamp(img_tuple[1].to_time())

        # convert object to string
        time_str = py_time.strftime("%d-%m-%Y_%H:%M:%S")
        img_str = "/root/photos/leo_" + time_str + ".jpg"

        cv2.imwrite(img_str, img)
