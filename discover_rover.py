import rospy
from time import perf_counter
from geometry_msgs.msg import Twist


class Rover:
    def __init__(self, name):
        self.name = name
        
    def drive(linear_vel, angular_vel, duration):
        twist = Twist()
        pub = rospy.Publisher("/cmd_vel", Twist, queue_size=20)
        
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        start_time = perf_counter()
                
        while perf_counter() - start_time <= duration:
            pub.publish(twist)

    def drive(linear_vel, angular_vel):
        drive(linear_vel, angular_vel, 1)

    def move_forward(velocity, duration):
        drive(velocity, 0, duration)

    def move_forward(velocity):
        drive(distance, 0)

    def move_backward(velocity, duration):
        drive(-velocity, 0, duration)

    def move_backward(velocity):
        drive(-velocity, 0)
    
    def turn_left(angle):
        drive(0, angle)

    def turn_right(angle):
        drive(0, -angle)
