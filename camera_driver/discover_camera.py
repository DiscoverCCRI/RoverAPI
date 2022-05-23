#!/usr/bin/env python3

import rospy
import PIL.Image as Img
from datetime import datetime
from sensor_msgs.msg import Image
from time import sleep
from os.path import exists
from os import mkdir


class Camera:
    """
    A class used to instantiate and use the raspicam on the LeoRover

    ...

    Attributes:
    -----------
    img_buffer:
        A list that contains the data of each image sent over the
        /camera/image_raw topic. Each image is stored in an array of
        unsigned 8-bit integers

    Methods:
    --------
    take_photo():
        Creates the ros node to capture the image, captures the int[] from the
        compressed imaged posted to the compressed image topic, and converts it
        to a .jpg image for the user
    __subscribe_to_image_topic():
        Creates a subscribe to subscribe to the /camera/image_raw/compressed
        topic created by the raspicam node. Then continues to run
        the function until the script is stopped
    __callback_get_image(message: CompressedImage):
        Gets the message from the /camera/image_raw/compressed topic and stores
        the image data to the image buffer
    __list_to_img(int_list: []) -> Img:
        Takes the list of unsigned 8-bit integers and converts it into an image
        object, then returns that image
    """

    def __init__(self):
        try:
            rospy.init_node("discover_rover")
        finally:
            self._img_buffer = []
            self.__subscribe_to_image_topic()

            if not exists("~/photos/"):
                mkdir("~/photos/")

            # allows the buffer to store an entire image before init is over
            sleep(1)

    def __subscribe_to_image_topic(self):
        rospy.Subscriber("/camera/image_raw", Image, self.__callback_get_image)

    def __callback_get_image(self, message: Image):
        time = datetime.now()

        if len(self._img_buffer) >= 30:
            self._img_buffer.pop(0)

        self._img_buffer.append((message.data, time))

    def take_photo(self):
        img_tuple = self._img_buffer[-1]
        img = self.__list_to_img(img_tuple[0])

        time_str = img_tuple[1].strftime("%d-%m-%Y_%H:%M:%S")
        img_str = "photos/leo_" + time_str + ".jpg"

        img.save(img_str)

    def __list_to_img(self, img_list: []) -> Img:
        bytestring = bytearray()

        for item in img_list:
            bytestring.append(item)

        bytesObj = bytes(bytestring)
        img = Img.frombytes("RGB", (640, 480), bytesObj)

        return img
