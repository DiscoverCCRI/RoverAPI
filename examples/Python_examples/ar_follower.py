#!/usr/bin/env python3
import math
import rospy

from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers

class leo_ar_follower(object):

    def __init__(self):
        self.last_marker_ts = rospy.Time()
        self.last_marker_position = None
        self.marker_angle = 0.0
        self.marker_distance = 0.0

        self.last_odom_ts = None
        self.odom_position = Vector3()
        self.odom_yaw = 0.0

        self.heading_pid_error_1 = 0.0
        self.heading_pid_ui_1 = 0.0
        self.distance_pid_error_1 = 0.0
        self.distance_pid_ui_1 = 0.0

        self.twist_cmd = Twist()
        self.follow_id = 0
        self.marker_timeout = rospy.Duration(0.5)

        self.goal_achieved = False

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.ar_pose_sub = rospy.Subscriber(
            "ar_pose_marker", AlvarMarkers, self.callback_ar_pose, queue_size=1
        )
        self.wheel_odom_sub = rospy.Subscriber(
            "wheel_odom_with_covariance",
            Odometry,
            self.callback_wheel_odom,
            queue_size=1,
        )

    def pid_distance(self):
        # PID controller to control the linear velocity based on distance
        ts = 0.2
        kp = 0.6
        ki = 4.15
        kd = 0.3
        desired_distance = 0.5
        error = desired_distance - self.marker_distance

        if error < 0.45:
            error = -error

        up = kp*error
        ui = ki*error + self.distance_pid_ui_1*ts
        ud = (kd/ts)*(error-self.distance_pid_error_1)
        u = up + ui + ud

        self.distance_pid_error_1 = error
        self.distance_pid_ui_1 = ui

        rospy.loginfo(f"ARTag_distance: {self.marker_distance}")

        return u
    
    def pid_heading(self):
        # PID controller to control the angular velocity based on heading
        ts = 0.2
        kp = 0.1
        ki = 0.20
        kd = 0.5
        error = self.marker_angle

        up = kp*error
        ui = ki*error + self.heading_pid_ui_1*ts
        ud = (kd/ts)*(error-self.heading_pid_error_1)
        u = up + ui + ud

        self.pid_error_1 = error
        self.heading_pid_ui_1 = ui

        return u

    def update_cmd(self):
        # Check for a timeout
        if self.last_marker_ts + self.marker_timeout < rospy.get_rostime():
            self.twist_cmd.linear.x = 0.0
            self.twist_cmd.angular.z = 0.0
            return

        lin_cmd = self.pid_distance()
        ang_cmd = self.pid_heading()

        self.twist_cmd.angular.z =  ang_cmd
        self.twist_cmd.linear.x = lin_cmd

    def update_marker_angle_distance(self):
        if self.last_marker_position:
            position_x = self.last_marker_position.x - self.odom_position.x
            position_y = self.last_marker_position.y - self.odom_position.y
            self.marker_angle = math.atan(position_y / position_x) - self.odom_yaw
            self.marker_distance = math.sqrt(
                position_x * position_x + position_y * position_y
            )

    def callback_ar_pose(self, msg):
        for marker in msg.markers:
            if marker.id != self.follow_id:
                continue
            if marker.header.stamp < self.last_marker_ts:
                rospy.logwarn_throttle(
                    3.0, "Got marker position with an older timestamp"
                )
                continue

            self.last_marker_ts = marker.header.stamp
            self.last_marker_position = marker.pose.pose.position
            self.odom_position = Vector3()
            self.odom_yaw = 0.0

            self.update_marker_angle_distance()

    def callback_wheel_odom(self, msg):
        if self.last_odom_ts:
            start_ts = max(self.last_odom_ts, self.last_marker_ts)

            end_ts = msg.header.stamp
            if end_ts < start_ts:
                rospy.logwarn(
                    "Reveived odometry has timestamp older than last marker position"
                )

            step_duration = (end_ts - start_ts).to_sec()

            # Integrate the velocity using rectangular rule
            self.odom_yaw += msg.twist.twist.angular.z * step_duration
            self.odom_position.x += (
                msg.twist.twist.linear.x * math.cos(self.odom_yaw) * step_duration
            )
            self.odom_position.y += (
                msg.twist.twist.linear.x * math.sin(self.odom_yaw) * step_duration
            )

            self.update_marker_angle_distance()

        self.last_odom_ts = msg.header.stamp

    def main(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_cmd()
            self.cmd_vel_pub.publish(self.twist_cmd)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("follow_ar_tag")
    ar_tag_follower = leo_ar_follower()
    ar_tag_follower.main()