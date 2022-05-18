#!/usr/bin/env python3

import rospy
from time import time
from geometry_msgs.msg import Twist
from math import radians


class Rover:

    """
    A class used to instantiate and drive LeoRovers

    ...

    Attributes:
    -----------
    none

    Methods:
    --------
    drive(linear_vel: float, angular_vel: float, duration: float)
        Drives the rover with both the linear (m/s) and angular (deg/s)
        velocities for a given number of seconds. The angular velocity is
        converted from deg/s to rad/s. The max linear velocity is ~0.5 m/s and
        the max angular velocity is ~45 deg/s.
    move_forward(velocity: float, duration: float)
        Drives the rover forward with a given velocity (m/s) for a given number
        of seconds. The max velocity is ~0.5 m/s.
    move_backward(velocity: float, duration: float)
        Drives the rover backward with a given velocity (m/s) for a given
        number of seconds. The max velocity is ~0.5 m/s.
    turn_left(angle: float, duration: float)
        Turns the rover to the left (counterclockwise) with a given angular
        velocity (deg/s) for a given number of seconds. The max velocity is
        ~45 deg/s.
    turn_right(angle: float, duration: float)
        Turns the rover to the left (clockwise) with a given angular velocity
        (deg/s) for a given number of seconds. The max velocity is ~45 deg/s.
    """

    def __init__(self):
        try:
            rospy.init_node("discover_rover")
        except:
            pass

    def drive(self, linear_vel: float, angular_vel: float, duration: float):
        twist = Twist()
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=20)
        angular_in_rad = radians(angular_vel)

        # stores values in the twist object
        twist.linear.x = linear_vel
        twist.angular.z = angular_in_rad

        start_time = time()

        # runs until duration has been reached
        while time() - start_time <= duration:
            pub.publish(twist)

    def move_forward(self, velocity, duration):
        self.drive(velocity, 0, duration)

    def move_backward(self, velocity, duration):
        self.drive(-velocity, 0, duration)

    def turn_left(self, angle, duration):
        self.drive(0, angle, duration)

    def turn_right(self, angle, duration):
        self.drive(0, -angle, duration)
