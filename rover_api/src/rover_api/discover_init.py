from rospy import init_node, Subscriber, Publisher, Time
from std_msgs.msg import Bool, String

LOG_FILE = "/experiment/log"

class Experimenter():
    
    def __init__(self, callback_func=None):
        self.init_node("discover_experiment")
        self.callback_func = callback_func
        self.finished_sub = Subscriber("/finished", Bool, self._callback)
        self.log_pub = Publisher("/experiment_log", String, queue_size=10)
        self.log_sub = Subscriber("/experiment_log", String, self._callback_log)

    def _callback(self, msg):
        if msg.data:
            self.callback_func()

    def _callback_log(self, msg):
        with open(LOG_FILE, "a") as outfile:
            outfile.write(msg)

    def set_callback(self, callback_func):
        self.callback_func = True

    def log(self, log_str: str):
        self.log_pub.publish(getTimeStr(Time.now(), "") + ": " + log_str)
