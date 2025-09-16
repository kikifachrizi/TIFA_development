#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

# Publisher for the marker to RViz
marker_pub = None

def calculate_grid_position(x, y, grid_size):
    """
    Menghitung posisi baris dan kolom dari koordinat x, y berdasarkan ukuran grid.
    """
    row = math.floor(y / grid_size)
    col = math.floor(x / grid_size)
    return row, col

def create_grid_marker(row, col, grid_size):
    """
    Membuat marker di posisi grid tertentu untuk ditampilkan di RViz.
    """
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "grid"
    marker.id = row * 100 + col  # Unique ID for each marker
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.pose.position.x = col * grid_size + grid_size / 2.0
    marker.pose.position.y = row * grid_size + grid_size / 2.0
    marker.pose.position.z = 0.5  # Slightly above ground
    marker.pose.orientation.w = 1.0

    # Marker scale (text size)
    marker.scale.z = 0.2

    # Marker color (red text)
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0

    # Marker text
    marker.text = "Row: {}, Col: {}".format(row, col)

    return marker

def lidar_callback(scan):
    """
    Callback function to process LIDAR data and publish marker to RViz.
    """
    global marker_pub
    grid_size = 2  # Size of each grid cell in meters

    for i, range_value in enumerate(scan.ranges):
        # Filter out invalid ranges
        if scan.range_min < range_value < scan.range_max:
            # Calculate the angle for this measurement
            angle = scan.angle_min + i * scan.angle_increment

            # Calculate the x and y coordinates of the detected object
            x = range_value * math.cos(angle)
            y = range_value * math.sin(angle)

            # Calculate the corresponding grid row and column
            row, col = calculate_grid_position(x, y, grid_size)

            # Create a marker for this grid position
            marker = create_grid_marker(row, col, grid_size)

            # Publish the marker to RViz
            marker_pub.publish(marker) # type: ignore

def main():
    global marker_pub

    # Initialize the ROS node
    rospy.init_node('lidar_grid_marker_node')

    # Create a publisher to send markers to RViz
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Subscribe to the LIDAR scan topic
    rospy.Subscriber('/scan', LaserScan, lidar_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
