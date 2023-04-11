from rospy import init_node, Subscriber
from std_msgs.msg import Bool


class ExperimentInitializer():
    """
    A class used to manage experiment-wide parameters.
    """
    
    def __init__(self, callback_func=None):
        """
        @brief The constructor for the ExperimentInitializer class. It starts up
        the discover ROS node if one is not already in operation. It then sets a
        callback function which will be called when the user is finished with their
        experiment, or if the rover goes into low-power mode.
        @param self: a reference to the current object.
        @param callback_func: a user-defined function that will be called when the
        experiment is over. It defaults to none, meaning no function will be called
        when the experiment finishes.
        @return: None
        """
        try:
            init_node("discover")
        finally:
            self.callback_func = callback_func
            self.finished_sub = Subscriber("/finished", Bool, self._callback)

    def _callback(self, msg):
        """
        @brief A callback function for the subscriber created in the class constructor.
        If the boolean passed to this function is True, the experiment is over, so the
        function calls the user's callback function.
        @param self: A reference to the current object.
        @param msg: A ROS Bool message sent from a publisher on the topic /finished.
        @return: None
        """
        if msg.data:
            self.callback_func()
     
    def set_callback(self, callback_func):
        """
        @brief A function allowing the user to set their callback function at a time other
        than construction of the object.
        @param self: A reference to the current object.
        @param callback_func: The new callback function to be used at the end of the experiment.
        @return: None
        """
        self.callback_func = callback_func
