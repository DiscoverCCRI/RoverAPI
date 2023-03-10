from os.path import exists
from os import mkdir
from sensor_msgs.msg import Image
from rospy import init_node, Time, Subscriber, sleep, loginfo
import cv2
from cv_bridge import CvBridge
from rover_api.discover_utils import Config, get_time_str
from rosbag import Bag


class Camera(Config):
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

    def __init__(self, subscribe=True, callback=None):
        try:
            loginfo("Camera initialized!")
        finally:
            self._img_buffer = []
            self._bag = None
            self._bag_open = False
            self.callback_func = callback
            if subscribe:
                self.subscribe_to_image_topic()
            if not exists("/experiment/photos/"):
                mkdir("/experiment/photos/")

            # allows the buffer to store an entire image before init is over
            sleep(1)
            super().__init__()

    def subscribe_to_image_topic(self):
        Subscriber("/camera/image_raw", Image, self.__callback_get_image)

    def __callback_get_image(self, message: Image):
        if self.callback_func is not None:
            self.callback_func()

        if(self._bag_open):
            self._bag.write("/camera/image_raw", message)

        # check if the buffer is full
        if len(self._img_buffer) >= 30:
            self._img_buffer.pop(0)

        # add the image message to the buffer
        self._img_buffer.append(message)

    def get_latest_image(self):
        return self._img_buffer[-1]

    def get_jpg(self):
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

    def start_recording(self):
        self._bag_open = True
        self._bag = Bag(get_time_str(Time.now(), ".bag"), 'w')

    def stop_recording(self):
        self._bag_open = False
        self._bag.close()

    def isAvailable(self):
        return super().isAvailable()

    def getInfo(self):
        return super().getInfo()
    
    def set_callback(self, func):
        self.callback_func = func
