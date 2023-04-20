from rospy import Time, Subscriber, Publisher, init_node, sleep
from time import time
from std_msgs.msg import Bool
from sensor_msgs.msg import CameraInfo, LaserScan
from nav_msgs.msg import Odometry
from datetime import datetime
from rospy_message_converter import message_converter
from rosnode import get_node_names, rosnode_info 
DATA_DIR = "/experiment/"


class Config:
    """
    This config class exists only as an abstract class
    through which common functionality between different
    RoverAPI classes is implemented.
    """

    def __init__(self):
        """
        This constructor sets up the Config class by finding which
        type of class is extending the config class.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        """
        try:
            init_node("discover")
        finally:
            self.sensor_type = ''
            self.sensor_info = None
            type_obj = str(type(self))
        
        if type_obj == "<class 'rover_api.discover_rover.Rover'>":
            self.sensor_type = "rover" 
        
        elif type_obj ==  "<class 'rover_api.discover_lidar.Lidar'>":
            self.sensor_type = "lidar"
        
        elif type_obj == "<class 'rover_api.discover_camera.Camera'>":
            self.sensor_type = "camera"

    def is_available(self):
        """
        Determines whether or not the given sensor is available.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        bool
            A boolean stating whether or not the sensor is available.
        """
        
        nodes = get_node_names()
        available_flag = False

        if self.sensor_type == "rover" and "/robot_state_publisher" in nodes:
            avaiable_flag = True
        
        elif self.sensor_type == "lidar" and "/rplidarNode" in nodes:
            available_flag = True
        
        # TODO: figure out why simulation does not start raspicam_node
        elif self.sensor_type == "camera": # and "/raspicam_node" in nodes:
           available_flag = True


        return available_flag
        
    def get_info(self):
        """
        Returns information about the given sensor. Information is returned
        in whatever format the sensor has chosen.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        sensor_info
           The sensor information in an unspecified format.
        """
        if self.sensor_type == "camera":
            Subscriber("/camera/camera_info", CameraInfo, self.
                       _callback_get_info)

        elif self.sensor_type == "lidar":
            Subscriber("/scan", LaserScan, self._callback_get_info)
        
        elif self.sensor_type == "rover":
            Subscriber("/wheel_odom_with_covariance", Odometry, 
                       self._callback_get_info)

        self.sensor_info = message_converter.convert_ros_message_to_dictionary(
                                                              self.sensor_info)
        return self.sensor_info

    def _callback_get_info(self, message):
        """
        This is a helper function that returns the information about the given sensor.
        
        Parameters
        ----------
        message : msg
            A ROS message containing the actual sensor info. The type of ROS message
            passed in depends on the type of sensor.
            
        Returns
        -------
        None
        """
        self.sensor_info = message 


def get_time_str(in_time: Time=None, extension: str='') -> str:
    """
    This function returns the current time this specific
    format: /experiment/day-month-year_hours:minutes:seconds.
    The prefix /experiment is included so files can be easily
    stored according to the time they were created.
    
    Parameters
    ----------
    in_time : Time 
        The current time according the ROS core. The default value is None.
    
    extension : str
        A string value that should pertain to a file extension. For rosbags it should be 
        .bag, for images it should be .jpg, etc.
    
    Returns
    -------
    str
        A string with the formatted time.
    
    Examples
    --------
    >>> from rover_api.discover_utils import get_time_str
    >>> from rover_api.discover_camera import Camera
    >>> def cb_func():
    >>>     print(f"New image at: {get_time_str()}")
    >>>
    >>> cam = Camera(callback=cb_func)
    """
    if in_time is not None:
        # convert time to a python datetime object
        py_time = datetime.fromtimestamp(in_time.to_time())
    else:
        py_time = datetime.fromtimestamp(time())

    # convert time object to string
    time_str = py_time.strftime("%d-%m-%Y_%H:%M:%S")
    return DATA_DIR + time_str + extension


def finish_experiment():
    """
    This function should be called whenever a user 
    experiment is finished. This will call the user's 
    finished callback function, doing whatever cleanup they
    may need. Additionally, this will be called when the rover
    goes into low power mode.
    
    Parameters
    ----------
    None
    
    Returns
    -------
    None
    
    Examples
    --------
    >>> from rover_api.discover_utils import get_time_str, finish_experiment
    >>> from rover_api.discover_camera import Camera
    >>> from rover_api.discover_init import ExperimentInitializer
    >>> def finished_cb():
    >>>     print(f"Finished at: {get_time_str()}")
    >>>
    >>> initializer = ExperimentInitializer(finished_cb)
    >>> cam = Camera()
    >>> finish_experiment()
    """
    pub = Publisher("/finished", Bool, queue_size=10)
    sleep(1)
    pub.publish(True)
