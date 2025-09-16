#!/usr/bin/env python3

import rospy
import math
import csv
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

# File CSV untuk menyimpan data
laser_csv_file = "laser_data.csv"
pose_csv_file = "pose_data.csv"

# Inisialisasi file CSV dan header
with open(laser_csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Laser_X", "Laser_Y"])

with open(pose_csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Pose_X", "Pose_Y"])

def laser_callback(msg):
    # Data LaserScan
    ranges = msg.ranges
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment

    # Menulis koordinat x, y ke CSV
    with open(laser_csv_file, mode="a", newline="") as file:
        writer = csv.writer(file)
        for i, r in enumerate(ranges):
            if r > 0:  # Filter nilai range yang valid
                angle = angle_min + i * angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                writer.writerow([x, y])
                rospy.loginfo(f"Laser Point {i}: x={x}, y={y}")

def pose_callback(msg):
    # Data Pose (x, y)
    x = msg.pose.position.x
    y = msg.pose.position.y

    # Menulis pose ke CSV
    with open(pose_csv_file, mode="a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([x, y])
        rospy.loginfo(f"Pose: x={x}, y={y}")

if __name__ == "__main__":
    rospy.init_node("hector_slam_listener")

    # Subscriber LaserScan
    rospy.Subscriber("/scan", LaserScan, laser_callback)

    # Subscriber Pose
    rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback)

    rospy.spin()
