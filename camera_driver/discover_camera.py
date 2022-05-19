#!/usr/bin/env python3

import rospy
import PIL.Image as Img
from datetime import datetime
from sensor_msgs.msg import Image
from time import sleep
import subproccess as sp


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
            self._video_on = False
            self._img_buffer = []
            self._video_stream = []
            self.__subscribe_to_image_topic()

            # allows the buffer to store an entire image before init is over
            sleep(1)

    def __subscribe_to_image_topic(self):
        subscriber = rospy.Subscriber("/camera/image_raw",
                     Image, self.__callback_get_image)

    def __callback_get_image(self, message: Image):
        self._img_buffer.append(message.data)

        if self._video_on:
            self._video_stream.append(message.data)

    def take_photo(self):
        img = self.__list_to_img(self._img_buffer[-1])

        time = datetime.now()
        time_str = "leo_cam_" + time.strftime("%d-%m-%Y_%H:%M:%S") + ".png"

        img.save(time_str)

    def __list_to_img(self, img_list: []) -> Img:
        bytestring = bytearray()

        for item in img_list:
            bytestring.append(item)

        bytesObj = bytes(bytestring)
        img = Img.frombytes("RGB", (640, 480), bytesObj)

        return img

    def start_video(self):
        self._video_on = True

    def stop_video(self):
        self._video_off = False

        for item in self._video_stream:
            item = self.__list_to_img(item)

        self.__save_video(self._video_stream)

    def __save_video(self, video_stream: []):
        cmd_out = ['ffmpeg', '-f', 'image2pipe', '-vcodec', 'png', '-r', '30',
                   '-i', '-', '-vcodec', 'png', '-qscale', '0',
                   '/root/scripts/vid.mp4']

        pipe = sp.Popen(cmd_out, stdin=sp.PIPE)

        for item in video_stream:
            item.save(pipe.stdin, 'PNG')

        pipe.stdin.close()
        pipe.wait()
