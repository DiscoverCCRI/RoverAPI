from rospy import init_node, Subscriber
from std_msgs.msg import Bool


class ExperimentInitializer():
    
    def __init__(self, callback_func=None):
        try:
            init_node("discover")
        finally:
            self.callback_func = callback_func
            self.finished_sub = Subscriber("/finished", Bool, self._callback)

    def _callback(self, msg):
        if msg.data:
            self.callback_func()
     
    def set_callback(self, callback_func):
        self.callback_func = callback_func
