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
from rospy import Subscriber, loginfo, init_node, Time, is_shutdown
from std_msgs.msg import Float32
from sensor_msgs.msg import CompressedImage
from rover_api.discover_utils import get_time_str
import docker


def callback_check_position(message: CompressedImage):
    # TODO: Change to use RTK
    num_same_pixels = 0.0
    pixel_total = float(len(message.data))

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
            num_same_pixels += 1.0

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
    # TODO: change to RTK
    Subscriber("/camera/image_raw/compressed", CompressedImage,
               callback_check_position)


def go_home():
    # TODO: define home for the rovers at each site, figure out AMCL
    pass


def kill_container(container):
    container.kill()
    loginfo(container.name + " has been killed")


def is_time_up() -> bool:
    return get_end_time() == get_time_str(Time.now(), "")


def get_end_time() -> str:
    # TODO: ask team how we want to store start and end times (env?)
    pass


def life_alert():
    # TODO: figure out how this is going to work
    pass


def save_data(container, src_dir: str, dest_file: str):
    # make sure the users know to put experiment data in ~/experiment
    with open(dest_file, "wb") as outfile:
        bits, stat = container.get_archive(src_dir)
        loginfo(stat)

        for chunk in bits:
            outfile.write(chunk)


def start_container(compose_file: str):
    # start with compose (API doesn't support docker compose)
    run("docker compose -f " + compose_file + " up -d", shell=True)
    client = docker.from_env()
    experiment_container = None

    # TODO: change to accomdate multi-container applications
    for container in client.containers.list():
        if container.name == "client":
            experiment_container = container

    # TODO: add case for container incorrectly starting
    if experiment_container is not None:
        return client, container


def upload_data(dest_link: str):
    # TODO: sort this thing out with team
    pass


def main():
    # start rosnode
    try:
        init_node("discover_control")
        loginfo("Control node started")
        client, container = start_container("docker-compose.yaml")
    except Exception:
        return

    # get arguments
    arguments = []
    with open(argv[1], "r") as infile:
        for line in infile:
            arguments.append(line.strip())

    # loop through all checks while rospy is active (which is always for Leo)
    while not (is_shutdown()):
        if is_time_up():
            save_data(container, "/root/experiment", "experiment_data.tar")
            # upload_data(dest_link)
            kill_container(container)
            return

        if not ("-nl" in arguments) and not ("--no-life-alert" in arguments):
            check_position()
        if not ("-np" in arguments) and not ("--no-power" in arguments):
            check_power()


if __name__ == "__main__":
    main()
