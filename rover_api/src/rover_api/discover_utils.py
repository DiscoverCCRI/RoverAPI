from rospy import Time
from datetime import datetime
from rosnode import get_node_names, rosnode_info 
DATA_DIR = "/experiment/"


# TODO: implement functionality for depth camera
class Config:

    def __init__(self):
        self.sensor_type = ''
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
        elif self.sensor_type == "camera" and "/raspicam_node" in nodes:
            available_flag = True

        return available_flag
        

def get_time_str(time: Time, extension: str) -> str:
    # convert time to a python datetime object
    py_time = datetime.fromtimestamp(time.to_time())

    # convert time object to string
    time_str = py_time.strftime("%d-%m-%Y_%H:%M:%S")
    return DATA_DIR + time_str + extension
