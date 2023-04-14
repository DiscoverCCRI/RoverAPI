from os.path import exists
from os import makedirs
from subprocess import run
from sensor_msgs.msg import Image
from rospy import init_node, Time, Subscriber, sleep, loginfo
import cv2
from cv_bridge import CvBridge
from rover_api.discover_utils import Config, get_time_str
from rosbag import Bag


class Camera(Config):
    """
    A class used to instantiate and use the raspicam on the LeoRover.
    The camera is a 5 megapixel webcam with a 170-degree field of view.
    Images are published at approximately 67 frames per second.
    """

    def __init__(self, subscribe=True, callback=None):
        """
        @brief The constructor for the Camera class. Sets initial values
        for attributes along with subscribing to image topic. Also creates
        the experimental directories if needed.
        @param self: The reference to the current object
        @param subscribe: A boolean value indicating whether or not to
        subscribe to the ROS image topic by default. Defaults to true.
        @param callback: A function set to be called every time new image
        data is published, and subscribe is True. Defaults to None.
        @return: None
        """
        try:
            loginfo("Camera initialized!")
        finally:
            self._img_buffer = []
            self._bag = None
            self._bag_open = False
            self.num_jpg = 0
            self.callback_func = callback
            if subscribe:
                self.subscribe_to_image_topic()
            if not exists("/experiment/photos/"):
                makedirs("/experiment/photos/")

            # allows the buffer to store an entire image before init is over
            sleep(1)
            super().__init__()

    def subscribe_to_image_topic(self):
        """
        @brief Subscribes to the ROS image topic published from the camera.
        If subscribe is set to False, call this function to get data from 
        the camera.
        @param self: A reference to the current object.
        @return: None
        """
        Subscriber("/camera/image_raw", Image, self.__callback_get_image)

    def __callback_get_image(self, message: Image):
        """
        @brief A helper function that is used to actually get image data.
        If the start_recording function has been called, this function will
        record image data to a rosbag file. If a callback function has been
        set, this function will call the callback. It appends the latest image
        data to an image buffer with a max length of 30 images.
        @param self: A reference to the current object.
        @param message: A sensor_msgs/Image object that corresponds to the
        latest image.
        @return: None
        """
        if self.callback_func is not None:
            self.callback_func()

        if(self._bag_open):
            self._bag.write("/camera/image_raw", message)

        # check if the buffer is full
        if len(self._img_buffer) >= 30:
            self._img_buffer.pop(0)

        # add the image message to the buffer
        self._img_buffer.append(message)

    def get_latest_image(self) -> Image:
        """
        @brief A function that returns the latest image in the form of 
        a sensor_msgs/Image object.
        @param self: A reference to the current object.
        @return: The latest image in the form of a sensor_msgs/Image object.
        """
        return self._img_buffer[-1]

    def get_jpg(self):
        """
        @brief A function that uses OpenCV's CvBridge to convert a 
        sensor_msgs/Image object to an OpenCV image object, which is then
        saved as an .jpg file in the /experiment/photos directory. Images
        are numbered sequentially from first to last.
        @param self: A reference to the current object.
        @return: None
        """
        # create the bridge to translate image types
        bridge = CvBridge()

        # get the image message
        img_msg = self._img_buffer[-1]

        # convert to an OpenCV image object
        img = bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')

        # get the file path to store image
        img_str = f"/experiment/photos/{self.num_jpg}.jpg"
         
        # update the number of jpgs
        self.num_jpg += 1
           
        # save image
        cv2.imwrite(img_str, img)

    def start_recording(self):
        """
        @brief A function that opens a new rosbag file to record all 
        ROS messages published on the /camera/image_raw topic.
        @param self: A reference to the current object.
        @return: None
        """
        self._bag_open = True
        self._bag = Bag(get_time_str(Time.now(), ".bag"), 'w')

    def stop_recording(self):
        """
        @brief A function that closes a previously opened rosbag file
        that has records of ROS messages published on the /camera/image_raw
        topic.
        @param self: A reference to the current object.
        @return: None
        """
        self._bag_open = False
        self._bag.close()

    def isAvailable(self) -> Bool:
        """
        @brief A function that returns whether or not the camera on the 
        rover is available.
        @param self: A reference to the current object.
        @return: A boolean value representing if the camera is available.
        """
        return super().isAvailable()

    def getInfo(self) -> str:
        """
        @brief A function that returns information about the camera on the
        rover.
        @param self: A reference to the current object.
        @return: A string containing information about the camera.
        """
        return super().getInfo()
    
    def set_callback(self, func):
        """
        @brief A function that sets the callback function to be called
        whenever new images from the camera are available
        @param self: A reference to the current object.
        @param func: The new callback function
        @return: None
        """
        self.callback_func = func
