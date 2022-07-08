#!/usr/bin/env python3

from sensor_msgs.msg import LaserScan, PointCloud2
from rospy import loginfo, sleep, Subscriber, init_node
import laser_geometry.laser_geometry as lg
from datetime import datetime
# import discover_depth_camera.DepthCamera as DepthCamera


class Lidar:
    def __init__(self):
        try:
            init_node("discover_rover")
            loginfo("Lidar initialized!")
        finally:
            self._scan_buffer = []
            self.__subscribe_to_scan()

            # give scan a chance to start publishing
            sleep(0.25)

    def __convert_to_pointcloud(self, message) -> PointCloud2:
        lp = lg.LaserProjection()

        return lp.projectLaser(message)

    def __subscribe_to_scan(self):
        Subscriber("/scan", LaserScan, self.__callback_get_scan)

    def __callback_get_scan(self, message: LaserScan):
        if len(self._scan_buffer) >= 30:
            self._scan_buffer.pop(0)

        self._scan_buffer.append(message)

    def get_latest_scan(self):

        laser_msg = self._scan_buffer[-1]

        # convert time to a python datetime object
        py_time = datetime.fromtimestamp(laser_msg.header.stamp.to_time())

        # convert time object to string
        time_str = py_time.strftime("%d-%m-%Y_%H:%M:%S")
        file_name = "scan_" + time_str + ".scan"

        with open(file_name, 'w', encoding='utf-8') as outfile:
            outfile.write("Sequence: " + laser_msg.header.seq)
            outfile.write("\nStamp: " + laser_msg.header.stamp)
            outfile.write("\nFrame ID: " + laser_msg.header.frame_id)
            outfile.write("\nAngle Min: " + laser_msg.angle_min)
            outfile.write("\nAngle Max: " + laser_msg.angle_max)
            outfile.write("\nAngle Increment: " + laser_msg.angle_increment)
            outfile.write("\nTime Increment: " + laser_msg.time_increment)
            outfile.write("\nScan Time: " + laser_msg.scan_time)
            outfile.write("\nRange Min: " + laser_msg.range_min)
            outfile.write("\nRange Max: " + laser_msg.range_max)
            outfile.write("\nRanges: " + laser_msg.ranges)
            outfile.write("\nIntensities: " + laser_msg.intensities)
