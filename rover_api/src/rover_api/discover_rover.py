from math import radians
from os.path import exists
from os import makedirs
from rospy import Time, init_node, Publisher, Subscriber, get_time, loginfo
from geometry_msgs.msg import Twist
from rosbag import Bag
from rover_api.discover_utils import get_time_str, Config


class Rover(Config):

    """
    A class used to instantiate and drive LeoRovers.
    The LeoRovers are four-wheel drive, operating at a maximum linear speed
    of 0.4 meters per second, and a maximum angular speed of 60 degrees per 
    second.
    """

    def __init__(self, subscribe=False, callback_func=None):
        """
        The constructor for the Rover class. 
        
        Sets initial values for attributes.
        
        Parameters
        ----------
        subscribe : bool 
            A boolean value indicating whether or not to subscribe to the ROS 
            cmd_vel topic by default. Defaults to False.
        callback : function 
            A function set to be called every time the rover moves
            and subscribe is True. Defaults to None.
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_rover import Rover
        >>> rover = Rover()
        """
        try:
            loginfo("Rover initialized!")
        finally:
            self._bag_open = False
            self.callback_func = callback_func
            if subscribe:
                self.__subscribe_to_vel()
            
            if not exists("/experiment/photos/"):
                makedirs("/experiment/photos/")
           
            self._rosbag = None
            self.subscribe_to_vel_topic()
            super().__init__() 

    def subscribe_to_vel_topic(self):
        """
        This function subscribes to the cmd_vel topic. This function only needs
        to be called if there is a callback that should be called whenever
        the rover moves and/or data needs to be saved to a rosbag. In either
        event, this function must be called beforehand,or subcribe must be set to
        True in the constructor.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_rover import Rover
        >>> rover = Rover()
        >>> rover.subscribe_to_vel_topic()
        """
        Subscriber("/cmd_vel", Twist, self.__callback_get_vel)
        
    def __callback_get_vel(self, msg: Twist):
        """
        This is a helper function to enable cmd_vel data to be written to an
        open rosbag file or to enable a callback function.
        
        Parameters
        ----------
        msg : Twist
            A ROS Twist object that contains data about the rover's movement.
        
        Returns
        -------Update discover_lidar.py
        None
        """
        if self.callback_func is not None:
            self.callback_func()

        if self._bag_open:
            self._rosbag.write("/cmd_vel", msg)
       
    def drive(self, linear_vel: float, angular_vel: float, duration: float):
        """ 
        This function allows the rover to drive, either linearly or angularly.
        
        Parameters
        ----------
        linear_vel : float
            The value in meters per second, of which the rover should move
            linearly (forward or backward). The maximum accepted value is 0.4 meters per second.
        angular_vel: float
            The value in degrees per second, of which the rover should move
            angularly (turn). The maximum accepted value is 60 degrees per second.
        duration : float
            The number of seconds for which the rover should execute given velocities.
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_rover import Rover
        >>> rover = Rover()
        >>> rover.drive(0.15, 10, 10)
        >>> rover.drive( -0.15, -10, 10)
        """
        twist = Twist()
        pub = Publisher("/cmd_vel", Twist, queue_size=20)
        angular_in_rad = radians(angular_vel)

        # stores values in the twist object
        twist.linear.x = linear_vel
        twist.angular.z = angular_in_rad

        start_time = get_time()

        # runs until duration has been reached
        while get_time() - start_time <= duration:
            pub.publish(twist)

    def move_forward(self, velocity: float, duration: float):
        """
        A function that allows the rover to move forward.
        
        Parameters
        ----------
        velocity : float
            The value in meters per second, for which the
            rover should move forward. Negative values will move the rover
            in reverse. The maximum accepted value is 0.4 meters per second.
        duration : float
            The number of seconds for which the rover should drive forward.
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_rover import Rover
        >>> rover = Rover()
        >>> rover.move_forward(.10, 10)
        """
        self.drive(velocity, 0, duration)

    def move_backward(self, velocity: float, duration: float):
        """
        A function that allows the rover to move backward.
        
        Parameters
        ----------
        velocity : float
            The value in meters per second, for which the rover
            should move in reverse. Negative value will move the rover forward.
            The minimum accepted value is -0.4 meters per second.
        duration : floatUpdate discover_lidar.py
            The number of seconds for which the rover should drive backward.
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_rover import Rover
        >>> rover = Rover()
        >>> rover.move_backward(.10, 10)
        """
        self.drive(-velocity, 0, duration)

    def turn_left(self, angle: float, duration: float):
        """
        A function that allows the rover to turn left.
        
        Parameters
        ----------
        velocity : float 
            The value in degrees per second, for which the rover
            should turn left. Negative value turn the rover to the right.
            The minimum accepted value is 60 degrees per second.
        duration : float
            The number of seconds for which the rover should turn left.
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_rover import Rover
        >>> rover = Rover()
        >>> rover.turn_left(10, 10)
        """
        self.drive(0, angle, duration)

    def turn_right(self, angle: float, duration: float):
        """
        A function that allows the rover to turn right.
        
        Parameters
        ----------
        velocity : float 
            The value in degrees per second, for which the rover
            should turn right. Negative value turn the rover to the left.
            The minimum accepted value is 60 degrees per second.
        duration : float
            The number of seconds for which the rover should turn right.
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_rover import Rover
        >>> rover = Rover()
        >>> rover.turn_right(10, 10)
        """
        self.drive(0, -angle, duration)
        
    def start_recording(self):
        """
        A function that opens a new rosbag file to record all 
        ROS messages published on the /cmd_vel topic. For this to work
        it must be called in conjunction with subscribe_to_vel_topic(),
        or subscribe must be set to True in the constructor.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_rover import Rover
        >>> rover = Rover()
        >>> rover.subscribe_to_vel_topic()
        >>> rover.start_recording()Update discover_lidar.py
        """
        self._bag_open = True
        self._rosbag = Bag(get_time_str(Time.now(), "_vel.bag"), 'w')

    def stop_recording(self):
        """
        A function that closes a previously opened rosbag file
        that has records of ROS messages published on the /cmd_vel
        topic.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_rover import Rover
        >>> rover = Rover()
        >>> rover.subscribe_to_vel_topic()
        >>> rover.start_recording()
        >>> rover.move_forward(.10, 10)
        >>> rover.stop_recording()
        """
        self._bag_open = False
        self._rosbag.close()

    def get_info(self):
        """
        A function that returns information about the rover.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_rover import Rover
        >>> rover = Rover()
        >>> info = rover.get_info()
        """
        return super().get_info()

    def __is_available(self):
        """
        A function that returns whether or not the rover is available.
        
        Parameters
        ----------
        None
        
        Returns
        -------
        bool 
            A boolean value representing if the rover is available.
        """
        return super().is_available()
    
    def set_callback(self, func):
        """
        A function that sets the callback function to be called whenever the rover moves.
        For the callback to be called, subscribe must be set to True in the constructor, or
        subscribe_to_vel_topic() must have been previously called.
        
        Parameters
        ----------
        func : function
            This is the new callback function.
        
        Returns
        -------
        None
        
        Examples
        --------
        >>> from rover_api.discover_rover import Rover
        >>> def cb_func():
        >>>     print("The rover is moving")
        >>>
        >>> rover = Rover()
        >>> rover.set_callback(cb_func)
        >>> rover.subscribe_to_vel_topic()
        """
        self.callback_func = func

