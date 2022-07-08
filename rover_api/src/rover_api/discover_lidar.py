#!/usr/bin/env python3

from sensor_msgs.msg import LaserScan, PointCloud2
from rospy import loginfo, sleep, Subscriber, init_node
import laser_geometry.laser_geometry as lg
import discover_depth_camera.DepthCamera as DepthCamera


class Lidar:
    def __init__(self):
        try:
            init_node("discover_rover")
            loginfo("Lidar initialized!")
        finally:
            pass

    def __convert_to_pointcloud(self, message) -> PointCloud2:
        lp = lg.LaserProjection()
        
        return lp.projectLaser(message)

    def save_as_pcd(self, pointcloud):


