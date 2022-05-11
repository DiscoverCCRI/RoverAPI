#!/usr/bin/env python3

""" Examples

This script contains examples of how to use the Rover object contained
in the RoverAPI library. For script to work, the user most have DiscoverCCRI's
RoverAPI installed on their system, as well as ROS(robot operating system).
This program simply drives the LeoRover back and forth

"""

from discover_rover import Rover

def main():
    leo = Rover("leo")

    leo.turn_left(41, 4)
    leo.turn_right(41, 4)
    leo.move_forward(.3048, 2)
    leo.move_backward(.3048, 2)
    leo.drive(-.3048, -7, 5)
    leo.drive(.3048, 7, 5)


if __name__ == "__main__":
    main()
