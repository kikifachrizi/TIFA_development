#!/usr/bin/env python3
import rospy
import csv
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def callback(point_cloud):
    print("a")
    with open('slam_cloud.csv', mode='w') as file:
        writer = csv.writer(file)
        writer.writerow(['x', 'y', 'z'])

        # Konversi data PointCloud2 ke list
        for point in pc2.read_points(point_cloud, skip_nans=True):
            writer.writerow([point[0], point[1], point[2]])

    rospy.loginfo("Data saved to slam_cloud.csv")

if __name__ == "__main__":
    rospy.init_node('slam_cloud_to_csv')
    rospy.Subscriber('/slam_cloud', PointCloud2, callback)
    rospy.spin()
