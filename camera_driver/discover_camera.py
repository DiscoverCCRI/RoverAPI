#!/usr/bin/env python3

from io import BytesIO
from rospy import init_node, Subscriber, sleep, loginfo
import PIL.Image as Img
from datetime import datetime
from sensor_msgs.msg import CompressedImage
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
        /camera/image_raw/compressed topic. Each image is stored in an array of
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
            init_node("discover_rover")
            loginfo("Rover node started!")
        finally:
            self._img_buffer = []
            self.__subscribe_to_image_topic()

            if not exists("/root/photos/"):
                mkdir("/root/photos/")

            # allows the buffer to store an entire image before init is over
            sleep(1)

    def __subscribe_to_image_topic(self):
        Subscriber("/camera/image_raw/compressed",
                   CompressedImage, self.__callback_get_image)

    def __callback_get_image(self, message: CompressedImage):
        time = datetime.now()

        if len(self._img_buffer) >= 30:
            self._img_buffer.pop(0)

        # add a tuple containing the unsigned 8-bit integer data and time
        self._img_buffer.append((message.data, time))

    def take_photo(self):
        # get the time object
        img_tuple = self._img_buffer[-1]
        img = self.__list_to_img(img_tuple[0])

        # conver object to string
        time_str = img_tuple[1].strftime("%d-%m-%Y_%H:%M:%S")
        img_str = "/root/photos/leo_" + time_str + ".jpg"

        img.save(img_str)

    def __list_to_img(self, img_list: []) -> Img:
        bytestring = bytearray()

        # get the byte form of the data
        for item in img_list:
            bytestring.append(item)

        img = Img.open(BytesIO(bytestring))

        return img
