#!/usr/bin/env python3

import rospy
import yaml
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Path file YAML
trajectory_file = "/home/deli/catkin_ws/src/deli_navigation/data/trajectory.yaml"

# Fungsi untuk membaca trajectory dari file YAML
def load_trajectory(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)
    return data['poses']

# Fungsi utama
def publish_path():
    rospy.init_node('path_publisher')
    pub = rospy.Publisher('/visualization_path', Path, queue_size=10)

    # Baca trajectory
    poses = load_trajectory(trajectory_file)

    # Buat pesan Path
    path = Path()
    path.header.frame_id = "map"  # Pastikan sesuai frame yang digunakan
    path.header.stamp = rospy.Time.now()

    for pose in poses:
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = pose['position']['x']
        pose_stamped.pose.position.y = pose['position']['y']
        pose_stamped.pose.position.z = pose['position']['z']
        pose_stamped.pose.orientation.x = pose['orientation']['x']
        pose_stamped.pose.orientation.y = pose['orientation']['y']
        pose_stamped.pose.orientation.z = pose['orientation']['z']
        pose_stamped.pose.orientation.w = pose['orientation']['w']
        path.poses.append(pose_stamped)

    rate = rospy.Rate(1)  # Frekuensi 1 Hz
    while not rospy.is_shutdown():
        pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
