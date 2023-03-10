from sensor_msgs.msg import LaserScan, PointCloud2
from rospy import loginfo, sleep, Subscriber, init_node, Time, Publisher
import laser_geometry.laser_geometry as lg
from rosbag import Bag
import roslaunch
from subprocess import run
from rover_api.discover_utils import Config, get_time_str
from os import mkdir
from os.path import exists
from itertools import islice



class Lidar(Config):
    """
    A class for instantiating and using the rplidar a2
    ...
    Attributes:
    -----------
    scan_buffer: a buffer that can hold up to thirty LaserScan messages
    which are produced by the lidar device.
    bag_open: a boolean value representing if the rosbag is open and recording
    rosbag: a rosbag object used to store the messages that are broadcasting
    on the scan topic so they can be replayed later using the rosbag command
    line tools.
    Methods:
    --------
    get_latest_scan(): saves the last scan in the buffer as a .scan text file
    start_recording(): opens the rosbag object, and then sets the recording
    flag to true.
    stop_recording(): closes the rosbag object, and sets the flag to false.
    convert_to_pointcloud(message: LaserScan) -> PointCloud2: takes in a
    LaserScan message and returns it as a PointCloud2 message.
    __subscribe_to_scan(): creates a subscriber to capture the LaserScan
    messages being broadcast by the lidar on the /scan topic.
    __callback_get_scan(message: LaserScan): saves the messages from the /scan
    topic to the buffer. It will also write them to the rosbag if the rosbag is
    currently open.
    """

    def __init__(self, callback=None, convert=False, subscribe=True):
        try:
            loginfo("Lidar initialized!")
        finally:
            super().__init__()
            self._scan_buffer = []
            self._bag_open = False
            self._rosbag = None
            self._convert = convert
            self._lp = None
            self._pc_pub = None
            self.callback_func = callback
            
            if self._convert:
                self._lp = lg.LaserProjection()
                self._pc_pub = Publisher("/pointcloud", PointCloud2, 
                                                                 queue_size=10)
            if subscribe:
                self.subscribe_to_scan()
            
            if not exists("/experiment"):
                mkdir("/experiment")
            
            # give scan a chance to start publishing
            sleep(0.25)

    def subscribe_to_scan(self):
        Subscriber("/scan", LaserScan, self.__callback_get_scan)

    def __callback_get_scan(self, message: LaserScan):
        pc2_msg = None 

        if self.callback_func is not None:
            self.callback_func()
        
        if self._convert:
            pc2_msg = self._lp.projectLaser(message)
            self._pc_pub.publish(pc2_msg)
            self._scan_buffer.append(pc2_msg)
        else:
            self._scan_buffer.append(message)

        if(self._bag_open):
            if self._convert:
                self._rosbag.write("/pointcloud", pc2_msg)
            else:
                self._rosbag.write("/scan", message)

        if len(self._scan_buffer) > 30:
            self._scan_buffer.pop(0)
        
    def get_latest_scan(self):
        return self._scan_buffer[-1] 
    
    def start_recording(self):
        self._bag_open = True
        self._rosbag = Bag(get_time_str(Time.now(), "_scan.bag"), 'w')

    def stop_recording(self):
        self._bag_open = False
        self._rosbag.close()
        if self._convert:
            run("rosrun pcl_ros bag_to_pcd /experiment/*.bag /pointcloud /experiment/pointclouds", shell=True)



    # TODO: figure out the different modes of the lidar, how to stop and start
    # external launch files, and if it is worth it to let the user change mode
    def isAvailable(self):
        return super().isAvailable()

    def getInfo(self):
        info_dict = super().getInfo()
        return dict(islice(info_dict.items(), 0, 8, 1))
