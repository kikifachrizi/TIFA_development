#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection

laser_proj = LaserProjection()

def scan_callback(msg):
    cloud_out = laser_proj.projectLaser(msg)
    cloud_out.header.frame_id = "base_link"
    cloud_pub.publish(cloud_out)

rospy.init_node('laserscan_to_pointcloud')
cloud_pub = rospy.Publisher('/filtered_cloud', PointCloud2, queue_size=10)
rospy.Subscriber('/scan', LaserScan, scan_callback)
rospy.spin()
