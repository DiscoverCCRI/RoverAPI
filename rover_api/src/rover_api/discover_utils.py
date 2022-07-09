from rospy import Time
from datetime import datetime


def get_time_str(time: Time, extension: str) -> str:
    # convert time to a python datetime object
    py_time = datetime.fromtimestamp(time.to_time())

    # convert time object to string
    time_str = py_time.strftime("%d-%m-%Y_%H:%M:%S")
    return time_str + extension
