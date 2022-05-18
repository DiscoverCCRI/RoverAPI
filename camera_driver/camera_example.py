#!/usr/bin/env python3

""" Examples

This script contains examples of how to use the Camera object contained
in the RoverAPI library. For script to work, the user most have DiscoverCCRI's
RoverAPI installed on their system, as well as ROS(robot operating system).
This program simply takes a few photos

"""

from discover_camera import Camera


def main():
    cam = Camera()
    cam.take_photo()


if __name__ == "__main__":
    main()
