<<<<<<< HEAD
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

            if not exists("photos/"):
                mkdir("photos/")

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
        img_str = "photos/leo_" + time_str + ".jpg"

        cv2.imwrite(img_str, img)
=======
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
        The constructor for the Camera class. 
        
        Sets initial values. for attributes along with subscribing to image 
        topic. Also creates the experimental directories if needed.
        
        Parameters
        ----------
        subscribe : bool 
            A boolean value indicating whether or not to subscribe to the ROS 
            image topic by default. Defaults to true.
        callback : function
            A function set to be called every time new image
            data is published, and subscribe is True. Defaults to None.
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_camera import Camera
        >>> def cb_func():
        >>>     print("New images for cam2")
        >>>
        >>> cam1 = Camera()
        >>> cam2 = Camera(False, cb_func)
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
        Subscribes to the ROS image topic published from the camera.
        If subscribe is set to False, call this function to get data from 
        the camera.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_camera import Camera
        >>> cam = Camera(False)
        >>> cam.subscribe_to_image_topic()
        """
        Subscriber("/camera/image_raw", Image, self.__callback_get_image)

    def __callback_get_image(self, message: Image):
        """
        A helper function that is used to actually get image data.
        If the start_recording function has been called, this function will
        record image data to a rosbag file. If a callback function has been
        set, this function will call the callback. It appends the latest image
        data to an image buffer with a max length of 30 images.
        
        Parameters
        ----------
        message : Image 
            A sensor_msgs/Image object that corresponds to the latest image.
            
        Returns
        -------
        None
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
        A function that returns the latest image in the form of 
        a sensor_msgs/Image object.
        
        
        Parameters
        ----------
        None
        
        Returns
        -------
        Image
            The latest image in the form of a sensor_msgs/Image object.
        
        Examples
        --------
        >>> from rover_api.discover_camera import Camera
        >>> from sensor_msgs.msg import Image
        >>> cam = Camera()
        >>> image1 = cam.get_latest_image()
        """
        return self._img_buffer[-1]

    def get_jpg(self):
        """ 
        A function that uses OpenCV's CvBridge to convert a 
        sensor_msgs/Image object to an OpenCV image object, which is then
        saved as an .jpg file in the /experiment/photos directory. Images
        are saved as /experiment/photos/<number_of_image>.jpg.
        
        Parameters
        ----------
        None        
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_camera import Camera
        >>> cam = Camera()
        >>> cam.get_jpg()
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
        A function that opens a new rosbag file to record all 
        ROS messages published on the /camera/image_raw topic.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_camera import Camera
        >>> cam = Camera()
        >>> cam.start_recording()
        """
        self._bag_open = True
        self._bag = Bag(get_time_str(Time.now(), ".bag"), 'w')

    def stop_recording(self):
        """
        A function that closes a previously opened rosbag file
        that has records of ROS messages published on the /camera/image_raw
        topic.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_camera import Camera
        >>> from time import sleep
        >>> cam = Camera()
        >>> cam.start_recording()
        >>> sleep(3)
        >>> cam.stop_recording()
        """
        self._bag_open = False
        self._bag.close()

    def is_available(self) -> bool:
        """
        A function that returns whether or not the camera on the 
        rover is available.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        bool
            A boolean value representing if the camera is available.
        
        Examples
        --------
        >>> from rover_api.discover_camera import Camera
        >>> cam = Camera(subscribe=False)
        >>> if cam.is_available():
        >>>     cam.subscribe_to_image_topic()
        """
        return super().is_available()

    def get_info(self) -> str:
        """
        A function that returns information about the camera on the
        rover.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        str
            A string containing information about the camera.
        
        Examples
        --------
        >>> from rover_api.discover_camera import Camera
        >>> cam = Camera()
        >>> info = cam.get_info()
        """
        return super().get_info()
    
    def set_callback(self, func):
        """
        A function that sets the callback function to be called
        whenever new images from the camera are available. The new callback
        function will not be called unless subscribe is set to True, or 
        subscribe_to_image_topic() has been called.
        
        Parameters
        ----------
        func : function
            The new callback function
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_camera import Camera
        >>> def cb_func():
        >>>     print("New image published")
        >>> 
        >>> cam = Camera(False, None)
        >>> cam.set_callback(cb_func)
        >>> cam.subscribe_to_image_topic()
        """
        self.callback_func = func
>>>>>>> origin/main
