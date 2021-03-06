from os.path import exists
from os import mkdir
from sensor_msgs.msg import Image
from rospy import init_node, Subscriber, sleep, loginfo
import cv2
from cv_bridge import CvBridge
from rover_api.discover_utils import get_time_str


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

            if not exists("photos/"):
                mkdir("photos/")

            # allows the buffer to store an entire image before init is over
            sleep(1)

    def __subscribe_to_image_topic(self):
        Subscriber("/camera/image_raw", Image, self.__callback_get_image)

    def __callback_get_image(self, message: Image):
        # check if the buffer is full
        if len(self._img_buffer) >= 30:
            self._img_buffer.pop(0)

        # add the image message to the buffer
        self._img_buffer.append(message)

    def take_photo(self):
        # create the bridge to translate image types
        bridge = CvBridge()

        # get the image message
        img_msg = self._img_buffer[-1]

        # convert to an OpenCV image object
        img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')

        # convert time to a python datetime object
        img_str = get_time_str(img_msg.header.stamp, ".jpg")

        # save image
        cv2.imwrite("photos/" + img_str, img)
