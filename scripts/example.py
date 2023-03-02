#!/usr/bin/env python3

from rover_api.discover_rover import Rover
from rover_api.discover_camera import Camera
from rover_api.discover_lidar import Lidar
from rover_api.discover_init import ExperimentInitializer
from rover_api.discover_utils import finish_experiment


def finished_cb():

    # Temporary Stub Return
    pass


def main():

    # initialize the experiment with a function that will be called when it
    # is over
    init = ExperimentInitializer(finished_cb())
    
    # initialize the objects to control the hardware
    rover = Rover()
    lidar = Lidar()
    cam = Camera()


    # take a rosbag recording of the lidar data
    lidar.start_recording()

    # drive in a 1 meter square
    for i in range(4):
        # drive forward at a rate of 0.2 m/s for 5s
        rover.move_forward(0.2, 5)
        
        # save an image of what the rover sees
        cam.get_jpg()

        # turn right at a rate of 15 deg/s for 6s
        rover.turn_right(15, 6) 


    # stop the recording
    lidar.stop_recording()

    # finish the experiment - this will automatically call our finished_cb func
    finish_experiment()


if __name__ == "__main__":
    main()
