#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from os.path import exists
from os import mkdir, remove
from importlib import import_module
from sys import argv


def callback_check_position(message: CompressedImage):
    num_same_pixels = 0
    pixel_total = len(message.data)

    if not exists("photos"):
        mkdir("photos")
        with open("__init__.py", "w") as outfile:
            outfile.write("import data")

    data_module = import_module("photos.data")
    prev_data = data_module.data

    for index in range(len(prev_data)):
        if prev_data[index] == message.data[index]:
            num_same_pixels += 1

    if num_same_pixels / pixel_total >= 0.97:
        life_alert()

    remove("photos/data.py")
    with open("photos/data.py", "w") as outfile:
        outfile.write("data: ")
        outfile.write(str(message.data))


def callback_check_power(message: Float32):
    if message.data < 10.0:
        go_home()


def check_power():
    rospy.Subscriber("/battery", Float32, callback_check_power)


def check_position():
    rospy.Subscriber("/camera/image_raw/compressed", CompressedImage,
                     callback_check_position)


def go_home():
    pass


def life_alert():
    pass


def main():
    try:
        rospy.init_node("discover_control")
    finally:
        rospy.loginfo("Control node started")

    arguments = []
    with open(argv[1], "r") as infile:
        for line in infile:
            arguments.append(line.strip())

    if not ("-nl" in arguments) and not ("--no-life-alert" in arguments):
        check_position()
    check_power()


if __name__ == "__main__":
    main()
