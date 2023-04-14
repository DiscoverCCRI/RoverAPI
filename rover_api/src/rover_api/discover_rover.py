from math import radians
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
        @brief The constructor for the Rover class. Sets initial values
        for attributes.
        @param self: The reference to the current object
        @param subscribe: A boolean value indicating whether or not to
        subscribe to the ROS cmd_vel topic by default. Defaults to False.
        @param callback: A function set to be called every time the rover moves
        and subscribe is True. Defaults to None.
        @return: None
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
            self.subscribe_to_vel()
            super().__init__() 

    def subscribe_to_vel(self):
        """
        @brief This function subscribes to the cmd_vel topic. This function only needs
        to be called if there is a callback that should be called whenever
        the rover moves and/or data needs to be saved to a rosbag. In either
        event, this function must be called beforehand.
        @param self: A reference to the current object.
        @return: None
        """
        Subscriber("/cmd_vel", Twist, self.__callback_get_vel)
        
    def __callback_get_vel(self, msg: Twist):
        """
        @brief This is a helper function to enable cmd_vel data to be written to an
        open rosbag file or to enable a callback function.
        @param self: A reference to the current object.
        @param msg: A ROS Twist object that contains data about the rover's movement.
        @return: None
        """
        if self.callback_func is not None:
            self.callback_func()

        if self._bag_open:
            self._rosbag.write("/cmd_vel", msg)
       
    def drive(self, linear_vel: float, angular_vel: float, duration: float):
        """
        @brief This function allows the rover to drive, either linearly or angularly.
        @param self: A reference to the current object.
        @param linear_vel: The value in meters per second, of which the rover should move
        linearly (forward or backward). The maximum accepted value is 0.4 meters per second.
        @param angular_vel: The value in degrees per second, of which the rover should move
        angularly (turn). The maximum accepted value is 60 degrees per second.
        @param duration: The number of seconds for which the rover should execute given velocities.
        @return: None
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
        @brief A function that allows the rover to move forward.
        @param velocity: The value in meters per second, for which the
        rover should move forward. Negative values will move the rover
        in reverse. The maximum accepted value is 0.4 meters per second.
        @param duration: The number of seconds for which the rover should
        drive forward.
        @return: None
        """
        self.drive(velocity, 0, duration)

    def move_backward(self, velocity: float, duration: float):
        """
        @brief A function that allows the rover to move backward.
        @param velocity: The value in meters per second, for which the rover
        should move in reverse. Negative value will move the rover forward.
        The minimum accepted value is -0.4 meters per second.
        @param duration: The number of seconds for which the rover should 
        drive backward.
        @return: None
        """
        self.drive(-velocity, 0, duration)

    def turn_left(self, angle: float, duration: float):
        """
        @brief A function that allows the rover to turn left.
        @param velocity: The value in degrees per second, for which the rover
        should turn left. Negative value turn the rover to the right.
        The minimum accepted value is 60 degrees per second.
        @param duration: The number of seconds for which the rover should 
        turn left.
        @return: None
        """
        self.drive(0, angle, duration)

    def turn_right(self, angle: float, duration: float):
         """
        @brief A function that allows the rover to turn right.
        @param velocity: The value in degrees per second, for which the rover
        should turn right. Negative value turn the rover to the left.
        The minimum accepted value is 60 degrees per second.
        @param duration: The number of seconds for which the rover should 
        turn right.
        @return: None
        """
        self.drive(0, -angle, duration)
        
    def start_recording(self):
        """
        @brief A function that opens a new rosbag file to record all 
        ROS messages published on the /cmd_vel topic.
        @param self: A reference to the current object.
        @return: None
        """
        self._bag_open = True
        self._rosbag = Bag(get_time_str(Time.now(), "_vel.bag"), 'w')

    def stop_recording(self):
        """
        @brief A function that closes a previously opened rosbag file
        that has records of ROS messages published on the /cmd_vel
        topic.
        @param self: A reference to the current object.
        @return: None
        """
        self._bag_open = False
        self._rosbag.close()

    def getInfo(self):
        """
        @brief A function that returns information about the 
        rover.
        @param self: A reference to the current object.
        @return: A string containing information about the rover.
        """
        return super().getInfo()

    def __isAvailable(self):
        """
        @brief A function that returns whether or not the 
        rover is available.
        @param self: A reference to the current object.
        @return: A boolean value representing if the rover is available.
        """
        return super().isAvailable()
    
     def set_callback(self, func):
        """
        @brief A function that sets the callback function to be called
        whenever the rover moves
        @param self: A reference to the current object.
        @param func: The new callback function
        @return: None
        """
        self.callback_func = func

