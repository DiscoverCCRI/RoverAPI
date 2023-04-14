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
        @brief This constructor sets up the Config class by finding which
        type of class is extending the config class.
        @param self: A reference to the current object.
        @return: None
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

    def isAvailable(self):
        """
        @brief Determines whether or not the given sensor is available.
        @param self: A reference to the current object.
        @return: None
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
        
    def getInfo(self):
        """
        @brief This function returns info about each individual 
        sensor.
        @param self: A reference to the current object.
        @return: None
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
        @brief This is a helper function that returns the
        information about the given sensor.
        @param self: A reference to the current object.
        @param message: A ROS message containing the actual
        sensor info.
        @return: None
        """
        self.sensor_info = message 

# TODO: just make this current time no matter what
def get_time_str(in_time: Time=None, extension: str='') -> str:
    """
    @brief This function returns the current time this specific
    format: /experiment/day-month-year_hours:minutes:seconds.
    The prefix /experiment is included so files can be easily
    stored according to the time they were created.
    @param in_time: The current time according the ROS core.
    The default value is None.
    @extension extensions: A string value that should pertain
    to a file extension. For rosbags it should be .bag, for images
    it should be .jpg, etc.
    @return: A string with the formatted time.
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
    @brief This function should be called whenever a user 
    experiment is finished. This will call the user's 
    finished callback function, doing whatever cleanup they
    may need. Additionally, this will be called when the rover
    goes into low power mode.
    @return: None
    """
    pub = Publisher("/finished", Bool, queue_size=10)
    sleep(1)
    pub.publish(True)
