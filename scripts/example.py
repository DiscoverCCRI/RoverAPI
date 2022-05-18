#!usr/bin/env python3

from rover_api.discover_rover import Rover
from rover_api.discover_camera import Camera


def main():
    rover = Rover()
    cam = Camera()

    cam.take_photo()
    rover.move_forward(.3048, 3)
    cam.take_photo()
    rover.turn_left(45, 2)
    rover.move_backward(.3048, 2)
    rover.turn_right(30, 3)
    rover.drive(-.25, 15, 7)
    cam.take_photo()


if __name__ == "__main__":
    main()
