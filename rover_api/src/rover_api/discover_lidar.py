from sensor_msgs.msg import LaserScan, PointCloud2
from rospy import loginfo, sleep, Subscriber, init_node, Time
import laser_geometry.laser_geometry as lg
from rosbag import Bag
from discover_utils import get_time_str
# import discover_depth_camera.DepthCamera as DepthCamera


class Lidar:
    def __init__(self):
        try:
            init_node("discover_rover")
            loginfo("Lidar initialized!")
        finally:
            self._scan_buffer = []
            self._bag_open = False
            self._rosbag = None
            self.__subscribe_to_scan()

            # give scan a chance to start publishing
            sleep(0.25)

    def __convert_to_pointcloud(self, message) -> PointCloud2:
        lp = lg.LaserProjection()

        return lp.projectLaser(message)

    def __subscribe_to_scan(self):
        Subscriber("/scan", LaserScan, self.__callback_get_scan)

    def __callback_get_scan(self, message: LaserScan):
        if(self._bag_open):
            self._rosbag.write("/scan", message)

        if len(self._scan_buffer) >= 30:
            self._scan_buffer.pop(0)

        self._scan_buffer.append(message)

    def get_latest_scan(self):

        laser_msg = self._scan_buffer[-1]

        file_name = get_time_str(laser_msg.header.stamp, ".scan")

        with open(file_name, 'w', encoding='utf-8') as outfile:
            outfile.write("Sequence: " + str(laser_msg.header.seq))
            outfile.write("\nStamp: " + str(laser_msg.header.stamp))
            outfile.write("\nFrame ID: " + str(laser_msg.header.frame_id))
            outfile.write("\nAngle Min: " + str(laser_msg.angle_min))
            outfile.write("\nAngle Max: " + str(laser_msg.angle_max))
            outfile.write("\nAngle Increment: "
                          + str(laser_msg.angle_increment))
            outfile.write("\nTime Increment: " + str(laser_msg.time_increment))
            outfile.write("\nScan Time: " + str(laser_msg.scan_time))
            outfile.write("\nRange Min: " + str(laser_msg.range_min))
            outfile.write("\nRange Max: " + str(laser_msg.range_max))
            outfile.write("\nRanges: " + str(laser_msg.ranges))
            outfile.write("\nIntensities: " + str(laser_msg.intensities))

    def start_recording(self):
        self._bag_open = True
        self._rosbag = Bag(get_time_str(Time.now(), ".bag"), 'w')

    def stop_recording(self):
        self._bag_open = False
        self._rosbag.close()
