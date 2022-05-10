import rospy

from geometry_msgs.msg import Twist


class Rover:
    

    def __init__(self, name):
        self.name = name
        rospy.init_node("rover", anonymous=True)

    def move(linear, angular, iterations):
        twist = Twist()
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=20)
        twist.linear.x = linear
        twist.angular.z = angular

        for index in range(iterations):
            pub.publish(twist)
            rospy.sleep(0.1)


    
            
        
