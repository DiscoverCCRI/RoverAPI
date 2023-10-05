from rospy import Subscriber, init_node, loginfo, get_rostime
import roslaunch
from visualization_msgs.msg import Marker


class Tracker():

    def __init__(self, subscribe=True, callback=None):
        """
        The constructor for the Tracker class. Also launches the alvar tracking
        node.

        Sets initial values for attributes.

        Parameters
        ----------
        subscribe : bool
            A boolean value indicating whether or not to subscribe to the ROS
            topic /visualization_marker by default.
        callback : function
            A callback function that is called whenever there are new messages
            published on the topic /visualization_marker and subscribe is True.

        Returns
        -------
        None

        Examples
        --------
        >>> from rover_api.discover_tracking import Tracker
        >>> tracker = Tracker()
        """
        try:
            init_node("discover")
        finally:
            loginfo("Tracker initialized!")
            uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            roslaunch.configure_logging(uuid)
            launch = roslaunch.parent.ROSLaunchParent(uuid,
              ["/home/pi/catkin_ws/src/leo_alvar_example/launch/alvar.launch"])
            launch.start()
            self.callback_func = callback
            self.marker = None
            self.marker_visible = False
            if subscribe:
                self.subscribe_to_marker_topic()

    def subscribe_to_marker_topic(self):
        """
        Subscribes to ROS /visualization_marker topic published by the alavar
        tracking node. If subscribe is set to False, call this function to get
        data from the tracking node

        Parameters
        ----------
        None

        Returns
        -------
        None

        Examples
        --------
        >>> from rover_api.discover_tracking import Tracker
        >>> tracker = Tracker(False)
        >>> tracker.subscribe_to_marker_topic()
        """
        Subscriber("/visualization_marker", Marker, self.__callback_get_marker)

    def __callback_get_marker(self, message: Marker):
        """
        A helper function that is used to actually get marker tracking data. If
        the recording function is called, this function will record data to a
        rosbag file. If a marker is visible, sets the flag to true, otherwise,
        it sets the flag to false.

        Parameters
        ----------
        message : Marker
            A visualization_msgs/Marker object that corresponds to marker
            tracking data.

        Returns
        -------
        None
        """
        if self.callback_func is not None:
            self.callback_func()

        self.marker = message

    def get_marker_visible(self) -> bool:
        """
        A function that returns the value of the marker_visible flag.

        Parameters
        ----------
        None

        Returns
        -------
        bool
            This boolean represents whether or not the rover's camera has
            detected an ARTag marker.

        Examples
        --------
        >>> from rover_api.discover_tracking import Tracker
        >>> tracker = Tracker()
        >>> is_visible = tracker.get_marker_visible()
        """

        if self.marker is None:
            return False

        return (self.marker.header.stamp.to_sec() -
                get_rostime().to_sec()) >= -0.4

    def get_marker_distance(self) -> float:
        """
        A function that returns the distance that the marker is from the camera. 
        If no marker is visible, returns None.

        Parameters
        ----------
        None

        Returns
        -------
        float
            The distance that the ARTag marker is from the camera in meters.
            Returns -1 if no ARTag marker is in the image.

        Examples
        --------
        >>> from rover_api.discover_tracking import Tracker
        >>> tracker = Tracker()
        >>> if tracker.get_marker_visible():
        >>>     print(tracker.get_marker_distance())
        """
        if self.get_marker_visible():
            return self.marker.pose.position.z

        return -1

    def get_marker_id(self) -> int:
        """
        A function that returns the ID of the ARTag marker. If no marker is
        visible, returns None

        Parameters
        ----------
        None

        Returns
        -------
        int
            The ID of the ARTag marker as a value between 0 and 17. Returns -1
            if no ARTag marker is in the image.

        Examples
        --------
        >>> from rover_api.discover_tracking import Tracker
        >>> tracker = Tracker()
        >>> if tracker.get_marker_visible():
        >>>     print(tracker.get_marker_id())
        """
        if self.get_marker_visible():
            return self.marker.id

        return -1
