#!/usr/bin/env python3

"""
Discover Rover Control

This script exercises control over the LeoRover for DiscoverCCRI.
It monitors all appropriate ros nodes and topics and can make the rover kill
all user code running in docker containers, call for help, and go home.

This script accepts a list of arguments that can change which parts of the
rover it script monitors.

This script should be combined with some sort of task-scheduler such as cron to
monitor on a schedule.
"""

from os.path import exists
from os import mkdir, remove
from importlib import import_module
from sys import argv
from subprocess import run
from rospy import Subscriber, loginfo, init_node
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage


def callback_check_position(message: CompressedImage):
    num_same_pixels = 0
    pixel_total = len(message.data)

    # check if there a directory for storing data from the camera
    # if not make it and an init file
    if not exists("photos"):
        mkdir("photos")
        with open("__init__.py", "w") as outfile:
            outfile.write("import data")

    data_module = import_module("photos.data")
    prev_data = data_module.data

    # check two images to see how many pixels they have in common
    for index in range(len(prev_data)):
        if prev_data[index] == message.data[index]:
            num_same_pixels += 1

    if num_same_pixels / pixel_total >= 0.97:
        life_alert()

    # cleanup
    remove("photos/data.py")
    with open("photos/data.py", "w") as outfile:
        outfile.write("data: ")
        outfile.write(str(message.data))


def callback_check_power(message: Float32):
    if message.data < 10.0:
        go_home()


def check_power():
    Subscriber("/battery", Float32, callback_check_power)


def check_position():
    Subscriber("/camera/image_raw/compressed", CompressedImage,
               callback_check_position)


def go_home():
    pass


def get_container_ids() -> []:
    ids = []
    container_id = ""

    # get the container id's of all running docker containers
    run("docker ps >> names.txt", shell=True, check=True)
    with open("names.txt", "r") as infile:
        for line in infile:
            if not("NAME" in line):
                for char in line:
                    if char == " ":
                        break
                    else:
                        container_id += char

            ids.append(container_id)
            container_id = ""

    # cleanup and return
    remove("names.txt")
    return(ids[1:])


def kill_containers(ids: []):
    for container in ids:
        run("docker kill " + container, shell=True, check=True)
        loginfo(container + " has been killed")


def life_alert():
    pass


def main():
    # start rosnode
    try:
        init_node("discover_control")
        loginfo("Control node started")
    except Exception:
        return

    # get arguments
    arguments = []
    with open(argv[1], "r") as infile:
        for line in infile:
            arguments.append(line.strip())

    kill_containers(get_container_ids())

    if not ("-nl" in arguments) and not ("--no-life-alert" in arguments):
        check_position()
    if not ("-np" in arguments) and not ("--no-power" in arguments):
        check_power()


if __name__ == "__main__":
    main()
