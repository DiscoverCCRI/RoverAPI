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

from sys import argv
from subprocess import run
from rospy import Publisher, Subscriber, loginfo, init_node, get_time, \
                  is_shutdown
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import CompressedImage
from rosgraph_msgs.msg import Log
import docker


DATA_DIR = "/experiment"
DATA_FILE = "experiment_data"
FINISHED = False


def callback_check_position(message):
    # TODO: Change to use RTK
    pass


def callback_check_power(message: Float32):
    return message.data

def callback_get_finished(message: Bool):
    global FINISHED
    FINISHED = message.data


def callback_rosout(message: Log):
    with open("/experiment/rover_experiment.log", "a") as outfile:
        outfile.write(f"{message.msg}\n")



def check_power():
    Subscriber("/battery", Float32, callback_check_power)


def check_position():
    # TODO: change to RTK
    pass


def go_home():
    # TODO: define home for the rovers at each site, figure out AMCL
    pass


def stop_container(container):
    loginfo(f"Experiment finished. {container.name} has been stopped and 
             removed.")
    loginfo(f"Battery power at {check_power()}V.")
    container.stop()
    container.remove()


def is_time_up(start_time) -> bool:
    return (get_time() - start_time) > 60


def get_end_time() -> str:
    # TODO: ask team how we want to store start and end times (env?)
    pass


def life_alert():
    # TODO: figure out how this is going to work
    pass


def save_data(container, src_dir: str, dest_file: str):
    # make sure the users know to put experiment data in /experiment
    with open(dest_file, "wb") as outfile:
        bits, stat = container.get_archive(src_dir, encode_stream=True)
        loginfo(stat)

        for chunk in bits:
            outfile.write(chunk)


def start_autoshutdown(container):
    loginfo(f"Power level at {check_power()}, starting automatic shutdown.")
    finished_pub = Publisher("/finished", Bool, queue_size=10)
    finished_pub.publish(True)



def start_container(compose_file: str):
    # start with compose (API doesn't support docker compose)
    run(f"docker compose -f {compose_file} up -d", shell=True)
    client = docker.from_env()
    experiment_container = None

    # TODO: change to accomdate multi-container applications
    for container in client.containers.list():
        if container.name == "client":
            experiment_container = container

    # TODO: add case for container incorrectly starting
    if experiment_container is not None:
        return client, container

    loginfo(f"Starting experiment with compose file {compose_file}.")
   
 
def upload_data(dest_link: str):
    # TODO: sort this thing out with team
    pass


def main():
    # start rosnode
    compose_file = "/home/pi/leorover-base-image/docker-compose.yaml"
    for argument in argv:
        if "-c" in argument:
            compose_file = argument.split("=")[1]
        elif "--compose-file" in argument:
            compose_file = argument.split("=")[1]

    try:
        init_node("discover_control")
        rosout_redirect = Subscriber("/rosout", Log, callback_log)
        loginfo("Control node started")
        loginfo(f"Battery power at {check_power()}V.")
        client, container = start_container(compose_file)
        finished_sub = Subscriber("/finished", Bool, callback_get_finished)
    except Exception:
        return

    # get arguments
    # with open(argv[1], "r") as infile:
    # for line in infile:
    # argument.append(line.strip())

    # loop through all checks while rospy is active (which is always for Leo)
    while not (is_shutdown()):
        if FINISHED:
            save_data(container, DATA_DIR, DATA_FILE + ".tar.gz")
            # upload_data(dest_link)
            stop_container(container)
            return

        else:
            if check_power() < 9.5:
                start_autoshutdown(container)    
	            

if __name__ == "__main__":
    main()
