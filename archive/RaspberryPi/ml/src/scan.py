#!/usr/bin/env python3

import rospy
import math
import csv
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped

laser_csv_file = "laser_data.csv"
# pose_csv_file = "pose_data.csv"

laser = []

with open(laser_csv_file, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Laser_X", "Laser_Y"])

# with open(pose_csv_file, mode="w", newline="") as file:
#     writer = csv.writer(file)
#     writer.writerow(["Pose_X", "Pose_Y"])

def laser_callback(msg):
    ranges = msg.ranges
    angle_min = msg.angle_min
    angle_increment = msg.angle_increment

    with open(laser_csv_file, mode="a", newline="") as file:
        writer = csv.writer(file)
        for i, r in enumerate(ranges):
            if r > 0:
                angle = angle_min + i * angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                # writer.writerow([x, y])
                laser.append([x, y])
                writer.writerow(laser)
                rospy.loginfo(f"Laser Point {i}: x={x}, y={y}")

# def pose_callback(msg):
#     x = msg.pose.position.x
#     y = msg.pose.position.y

#     with open(pose_csv_file, mode="a", newline="") as file:
#         writer = csv.writer(file)
#         writer.writerow([x, y])
#         rospy.loginfo(f"Pose: x={x}, y={y}")

if __name__ == "__main__":
    rospy.init_node("hector_slam_listener")

    rospy.Subscriber("/scan", LaserScan, laser_callback)

    # rospy.Subscriber("/slam_out_pose", PoseStamped, pose_callback)

    rospy.spin()
