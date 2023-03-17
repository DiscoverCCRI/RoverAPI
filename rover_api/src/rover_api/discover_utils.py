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

    def __init__(self):
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
       self.sensor_info = message 

# TODO: just make this current time no matter what
def get_time_str(in_time: Time=None, extension: str='') -> str:
    if in_time is not None:
        # convert time to a python datetime object
        py_time = datetime.fromtimestamp(in_time.to_time())
    else:
        py_time = datetime.fromtimestamp(time())

    # convert time object to string
    time_str = py_time.strftime("%d-%m-%Y_%H:%M:%S")
    return DATA_DIR + time_str + extension


def finish_experiment():
    
    pub = Publisher("/finished", Bool, queue_size=10)
    sleep(1)
    pub.publish(True)
