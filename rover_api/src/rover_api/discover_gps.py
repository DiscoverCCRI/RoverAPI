from rospy import loginfo, sleep, Subscriber, init_node
from rosbag import Bag
from sensor_msgs.msg import NatSavFix
from subprocess import run
from rover_api.discover_utils import get_time_str


class GPS:

    def __init__(self):
        try:
            loginfo("GPS listener started")
        finally:
            self.__subscribe_to_gps()
            self._fix_buffer = []

            sleep(1)

    def __subscribe_to_gps(self):
        Subscriber("/fix", NatSavFix, self.__callback_get_fix)

    def __callback_get_fix(self, msg):
        if len(self._fix_buffer) >= 30:
            self._fix_buffer.pop(0)

        self._fix_buffer.append(msg)
