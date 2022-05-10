#!/usr/bin/env python

from discover_rover import Rover

def main():
    leo = Rover()

    leo.drive(.3048, 7, 5)
    leo.turn_left(41, 4)
    leo.move_backward(.3048, 2)
    leo.move_forward(.3048, 3)


if __name__ == "__main__":
    main()
