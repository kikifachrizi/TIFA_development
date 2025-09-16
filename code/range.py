#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import math

class RPLidarReader:
    def __init__(self, target_angle_deg):
        self.target_angle_deg = target_angle_deg % 360  # Pastikan dalam rentang 0–359
        self.range_at_angle = None
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.loginfo(f"Menunggu data jarak pada sudut {self.target_angle_deg} derajat...")

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min  # dalam radian
        angle_increment = msg.angle_increment  # dalam radian
        ranges = msg.ranges

        # Ubah sudut target ke radian
        target_angle_rad = math.radians(0)

        # Hitung index dalam array ranges
        index = int((0 - angle_min) / angle_increment)

        # Validasi index
        if 0 <= index < len(ranges):
            distance = ranges[index]
            if math.isinf(distance) or math.isnan(distance):
                rospy.logwarn(f"Sudut {self.target_angle_deg}°: Tidak ada objek terdeteksi.")
            else:
                rospy.loginfo(f"Sudut {self.target_angle_deg}°: Jarak = {distance:.2f} meter")
        else:
            rospy.logwarn("Sudut di luar jangkauan data lidar")

if __name__ == "__main__":
    rospy.init_node("rplidar_reader")
    target_angle = float(input("Masukkan sudut (derajat): "))
    reader = RPLidarReader(target_angle)
    rospy.spin()
