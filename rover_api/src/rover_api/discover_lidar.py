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
    A class for using the RPLidar A2. This is a two-dimensional lidar that
    provides 360-degree range-finding capabilities. It operates at an 8k 
    sampling frequency, with a 12 meter scan range and 10 Hz rotational speed.
    """

    def __init__(self, callback=None, convert=False, subscribe=True):
        """The constructor for the Lidar class. 
        
        This function sets up all 
        attributes of the class, and subscribes to the proper topics to enable
        data collection from the lidar.
        
        Parameters
        ----------
        callback : function
            A function to be called whenever new data is published
            by the lidar. The default value is none.
        convert : bool
            A boolean value specifying whether to convert the two-
            dimensional laser scan data published by the lidar into pointcloud form.
            The default value is False.
        subscribe : bool
            A boolean value specifying whether to subscribe to, and
            collect data from, the topic published by the lidar. The default value is
            True.
        
        Returns
        -------
        None
       
        Examples
        --------
        >>> from rover_api.discover_lidar import Lidar
        >>> def cb_func():
        >>>      print("New lidar data")
        >>>
        >>> scanner1 = Lidar()
        >>> scanner2 = Lidar(cb_func, True, True)
        """
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
                self.subscribe_to_scan_topic()
            
            if not exists("/experiment"):
                mkdir("/experiment")
            
            # give scan a chance to start publishing
            sleep(0.25)

    def subscribe_to_scan_topic(self):
        """A function allowing the user to subscribe to the scan
        topic at a time other than object instantiation.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_lidar import Lidar
        >>> scanner = Lidar(subscribe=False)
        >>> scanner.subscribe_to_scan_topic()
        """
        Subscriber("/scan", LaserScan, self.__callback_get_scan)

    def __callback_get_scan(self, message: LaserScan):
        """A helper method to facilitate the passing of laserscan data
        from the /scan ROS topic to user-facing methods. It appropriately writes
        data to rosbags and/or converts to pointclouds if needed.
        
        Parameters
        ----------
        message : LaserScan
            A ROS LaserScan object generated by the lidar on the /scan topic.
        
        Returns
        -------
        None
        """
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
        """A method which returns the latest LaserScan message object.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        LaserScan
            The latest LaserScan message object in the scan buffer.
        
        Examples
        --------
        >>> from rover_api.discover_lidar import Lidar
        >>> from sensor_msgs.msg import LaserScan
        >>> scanner = Lidar()
        >>> sscan = scanner1.get_latest_scan()
        """
        return self._scan_buffer[-1] 
    
    def start_recording(self):
        """A method which starts recording data from the lidar to a rosbag.
        The bag is named /experiment/<time_at_start>_scan.bag.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_lidar import Lidar
        >>> scanner = Lidar()
        >>> scanner.start_recording()
        """
        self._bag_open = True
        self._rosbag = Bag(get_time_str(Time.now(), "_scan.bag"), 'w')

    def stop_recording(self):
        """A method which stops recording data from the lidar to a rosbag.
        If convert was set to true in the constructor, the rosbag is converted
        to a pointcloud.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_lidar import Lidar
        >>> from time import sleep
        >>> scanner = Lidar()
        >>> scanner.start_recording()
        >>> sleep(3)
        >>> scanner.stop_recording()
        """
        self._bag_open = False
        self._rosbag.close()
        if self._convert:
            run("rosrun pcl_ros bag_to_pcd /experiment/*.bag /pointcloud /experiment/pointclouds", shell=True)



    # TODO: figure out the different modes of the lidar, how to stop and start
    # external launch files, and if it is worth it to let the user change mode
    def is_available(self):
        """A method which lets the user know if the lidar is available on the rover.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        bool
            A boolean value specifying if the lidar is available for the user.
        
        Examples
        --------
        >>> from rover_api.discover_lidar import Lidar
        >>> scanner = Lidar(subscribe=False)
        >>> if scanner.is_available():
        >>>     scanner.subscribe_to_scan_topic()
        """
        return super().is_available()

    def get_info(self):
        """A method which gives the user info about the configuration of the lidar.
        Items like scan rate, and scan distance are included.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        str
            A string containing information about the lidar.
        
        Examples
        --------
        >>> from rover_api.discover_lidar import Lidar
        >>> scanner = Lidar()
        >>> info = scanner.get_info()
        """
        info_dict = super().get_info()
        return dict(islice(info_dict.items(), 0, 8, 1))
        
    def set_callback(self, func):
        """A function that sets the callback function to be called
        whenever new scans from the Lidar are available. The new callback
        function will not be called unless subscribe is set to True, or 
        subscribe_to_scan_topic() has been called.
        
        Parameters
        ----------
        func : function
            The new callback function
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_lidar import Lidar
        >>> def cb_func():
        >>>     print("New scan published")
        >>> 
        >>> scanner = Lidar(callback=None, subscribe=False)
        >>> scanner.set_callback(cb_func)
        >>> scanner.subscribe_to_scan_topic()
        """
        self.callback_func = func
