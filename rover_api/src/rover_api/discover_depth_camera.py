from os.path import exists
from os import mkdir
from sensor_msgs.msg import Image, PointCloud2
from rospy import sleep, Subscriber, init_node, loginfo
import cv2
from cv_bridge import CvBridge
from rover_api.discover_utils import get_time_str


class DepthCamera:
    """
    A class for instantiating and using the Intel RealSense depth camera.
    ...
    Attributes:
    -----------
    depth_buffer: a buffer of the 15 newest images published from the depth
    camera.
    pc_buffer: a buffer of the 15 newest point clouds published from the depth
    camera.
    Methods:
    --------
    take_depth_photo(): gets the newest image from the depth buffer and then
    saves it as .jpg file.
    get_point_cloud(): returns the newest point cloud from the buffer.
    __callback_get_depth(message: Image): saves the images published on the
    /camera/depth/image_rect_raw topic and saves them to the buffer.
    __callback_get_point_cloud(message: PointCloud2) : saves the point clouds
    published on the /camera/depth/color/points and saves them to the buffer.
    __subscibe_to_depth_image(): creates a subscriber and subscribes to the
    /camera/depth/image_rect_raw topic.
    __subscribe_to_point_cloud(): creates a subscriber and subscribes to the
    /camera/depth/color/points topic.
    """

    def __init__(self):
        try:
            init_node("discover_rover")
            loginfo("Depth Camera initialized!")
        finally:
            self._depth_buffer = []
            self._pc_buffer = []
            self.__subscribe_to_depth_image()
            self.__subscribe_to_point_cloud()

            if not exists("photos/"):
                mkdir("photos/")

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

        img_str = get_time_str(depth_img.header.stamp, ".jpg")

        cv2.imwrite("photos/" + img_str, img)

    def get_point_cloud(self) -> PointCloud2:
        return self._pc_buffer[-1]
