from sensor_msgs.msg import LaserScan, PointCloud2
from rospy import loginfo, sleep, Subscriber, init_node, Time
import laser_geometry.laser_geometry as lg
from rosbag import Bag
import roslaunch
from subprocess import run
from rover_api.discover_utils import Config, get_time_str
from os import mkdir
from os.path import exists
from itertools import islice
# import discover_depth_camera.DepthCamera as DepthCamera



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

    def __init__(self, subscribe=True, callback=None):
        try:
            loginfo("Lidar initialized!")
        finally:
            self._scan_buffer = []
            self._bag_open = False
            self._rosbag = None

            if subscribe:
                self.__subscribe_to_scan()
            self.callback_func = callback
            
            
            # self._map_launch = self.__init_launch()
            self.__subscribe_to_scan()   
            # TODO: figure out why this is in here
            # self._map_launch = self.__init_launch()
            # TODO: figure out why this is broken
            # if not exists("/experiment/maps/"):
            #     mkdir("/experiment/maps")

            # give scan a chance to start publishing
            sleep(0.25)

            super().__init__()

    def convert_to_pointcloud(self, message: LaserScan) -> PointCloud2:
        lp = lg.LaserProjection()

        return lp.projectLaser(message)

    def subscribe_to_scan(self):
        Subscriber("/scan", LaserScan, self.__callback_get_scan)

    def __callback_get_scan(self, message: LaserScan):
        if self.callback_func is not None:
            self.callback_func()

        if(self._bag_open):
            self._rosbag.write("/scan", message)

        if len(self._scan_buffer) >= 30:
            self._scan_buffer.pop(0)

        self._scan_buffer.append(message)

    def get_latest_scan(self):
        return self._scan_buffer[-1] 

    def __init_launch(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/root/catkin_ws/src"
                   + "/hector_slam/hector_slam_launch/launch/tutorial.launch"])
        return launch

    def start_mapping(self):
        self._map_launch.start()

    def stop_mapping(self):
        run("rosrun map_server map_saver -f maps/" + get_time_str(Time.now(), ""),
            shell=True)
        self._map_launch.shutdown()

    def start_recording(self):
        self._bag_open = True
        self._rosbag = Bag(get_time_str(Time.now(), "_scan.bag"), 'w')

    def stop_recording(self):
        self._bag_open = False
        self._rosbag.close()


    # TODO: figure out the different modes of the lidar, how to stop and start
    # external launch files, and if it is worth it to let the user change mode
    def isAvailable(self):
        return super().isAvailable()

    def getInfo(self):
        info_dict = super().getInfo()
        return dict(islice(info_dict.items(), 0, 8, 1))
