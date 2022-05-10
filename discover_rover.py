#!/usr/bin/env python3
import rospy
from time import perf_counter 
from geometry_msgs.msg import Twist
from math import radians

class Rover:
    def __init__(self, name):
        self.name = name

    def drive(self, linear_vel, angular_vel, duration):
        rospy.init_node("drive_rover")
        twist = Twist()
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=20)

        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        start_time = perf_counter()

        while perf_counter() - start_time <= duration:
            pub.publish(twist)

    def move_forward(self, velocity, duration):
        self.drive(velocity, 0, duration)

    def move_backward(self, velocity, duration):
        self.drive(-velocity, 0, duration)

    def turn_left(self, angle, duration):
        angle_in_rad = radians(angle)
        self.drive(0, angle_in_rad, duration)

    def turn_right(self, angle, duration):
        angle_in_rad = radians(angle)
        self.drive(0, -angle_in_rad, duration)
