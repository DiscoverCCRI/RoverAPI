#!/usr/bin/env python3

import rospy
import PIL.Image as Img
from io import BytesIO
from datetime import datetime
from sensor_msgs.msg import CompressedImage
from time import sleep


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
        topic created by the raspicam node. Then "spins" or continues to run
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

            # sleeps to allow the buffer to process an entire image upon init
            sleep(0.25)

    def __subscribe_to_image_topic(self):
        subscriber = rospy.Subscriber("/camera/image_raw/compressed",
                     CompressedImage, self.__callback_get_image)

    def __callback_get_image(self, message: CompressedImage):
        self._img_buffer.append(message.data)

    def take_photo(self):
        img = self.__list_to_img(self._img_buffer[-1])

        time = datetime.now()
        time_str = "leo_cam_" + time.strftime("%d-%m-%Y_%H:%M:%S") + ".jpg"

        img.save(time_str)

    def __list_to_img(self, img_list: []) -> Img:
        bytestring = bytearray()

        for item in img_list:
            bytestring.append(item)

        img = Img.open(BytesIO(bytestring))

        return img
